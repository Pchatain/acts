// This file is part of the Acts project.
//
// Copyright (C) 2019 CERN for the benefit of the Acts project
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "ACTFW/Io/Performance/TrackSeedingPerformanceWriter.hpp"

#include "ACTFW/EventData/GeometryContainers.hpp"
#include "ACTFW/EventData/IndexContainers.hpp"
#include "ACTFW/EventData/ProtoTrack.hpp"
#include "ACTFW/EventData/SimMultiTrajectory.hpp"
#include "ACTFW/EventData/SimParticle.hpp"
#include "ACTFW/Utilities/Paths.hpp"
#include "ACTFW/Utilities/Range.hpp"
#include "ACTFW/Validation/ProtoTrackClassification.hpp"
#include "Acts/EventData/MultiTrajectoryHelpers.hpp"
#include "Acts/EventData/TrackParameters.hpp"
#include "Acts/Plugins/Digitization/PlanarModuleCluster.hpp"
#include "Acts/Seeding/Seed.hpp"
#include "Acts/Seeding/SpacePoint.hpp"
#include "Acts/Utilities/Units.hpp"
#include "Acts/Utilities/detail/periodic.hpp"
#include <Acts/Utilities/Helpers.hpp>

#include <algorithm>
#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include <TFile.h>
#include <TTree.h>

namespace {
using Acts::VectorHelpers::eta;
using SimParticleContainer = FW::SimParticleContainer;
using HitParticlesMap = FW::IndexMultimap<ActsFatras::Barcode>;
using ProtoTrackContainer = FW::ProtoTrackContainer;
using ProtoTrack = FW::ProtoTrack;
}  // namespace

struct FW::TrackSeedingPerformanceWriter::Impl {
  Config cfg;
  TFile* file = nullptr;

  // per-track tree
  TTree* trkTree = nullptr;
  std::mutex trkMutex;
  // track identification
  ULong64_t trkEventId;
  ULong64_t trkTrackId;
  // track content
  // True seed = 0 and Fake = 1
  UShort_t trkTrue;  // TODO: Implement
  // number of hits on track
  UShort_t trkNumHits;
  // number of particles contained in the track
  UShort_t trkNumParticles;
  // track particle content; for each contributing particle, largest first
  std::vector<ULong64_t> trkParticleId;
  // total number of hits generated by this particle
  std::vector<UShort_t> trkParticleNumHitsTotal;
  // number of hits within this track
  std::vector<UShort_t> trkParticleNumHitsOnTrack;

  // per-particle tree
  TTree* prtTree = nullptr;
  std::mutex prtMutex;
  // particle identification
  ULong64_t prtEventId;
  ULong64_t prtParticleId;
  Int_t prtParticleType;
  // particle kinematics
  // vertex position in mm
  float prtVx, prtVy, prtVz, prtR;
  // vertex time in ns
  float prtVt;
  // particle momentum at production in GeV
  float prtPx, prtPy, prtPz, prtAbsP;
  // particle mass in GeV
  float prtM;
  // particle charge in e
  float prtQ;
  // particle reconstruction
  UShort_t prtNumHits;    // number of hits for this particle
  UShort_t prtNumTracks;  // number of tracks this particle was reconstructed in
  UShort_t prtNumTracksMajority;  // number of tracks reconstructed as majority
  // extra logger reference for the logging macros
  const Acts::Logger& _logger;

  Impl(Config&& c, const Acts::Logger& l) : cfg(std::move(c)), _logger(l) {
    if (cfg.inputParticles.empty()) {
      throw std::invalid_argument("Missing particles input collection");
    }
    if (cfg.inputHitParticlesMap.empty()) {
      throw std::invalid_argument("Missing hit-particles map input collection");
    }
    if (cfg.inputProtoSeeds.empty()) {
      throw std::invalid_argument("Missing proto seeds input collection");
    }
    if (cfg.outputFilename.empty()) {
      throw std::invalid_argument("Missing output filename");
    }
    if (cfg.outputIsML) {
      return;
    }
    // the output file can not be given externally since TFile accesses to the
    // same file from multiple threads are unsafe.
    // must always be opened internally
    auto path = joinPaths(cfg.outputDir, "TTrees_" + cfg.outputFilename);
    file = TFile::Open(path.c_str(), "RECREATE");
    if (not file) {
      throw std::invalid_argument("Could not open '" + path + "'");
    }

    // construct trees
    trkTree = new TTree("track_finder_tracks", "");
    trkTree->SetDirectory(file);
    trkTree->Branch("event_id", &trkEventId);
    trkTree->Branch("track_id", &trkTrackId);
    trkTree->Branch("size", &trkNumHits);
    trkTree->Branch("True_or_fake", &trkTrue);
    trkTree->Branch("nparticles", &trkNumParticles);
    trkTree->Branch("particle_id", &trkParticleId);
    trkTree->Branch("particle_nhits_total", &trkParticleNumHitsTotal);
    trkTree->Branch("particle_nhits_on_track", &trkParticleNumHitsOnTrack);
    prtTree = new TTree("track_finder_particles", "");
    prtTree->SetDirectory(file);
    prtTree->Branch("event_id", &prtEventId);
    prtTree->Branch("particle_id", &prtParticleId);
    prtTree->Branch("particle_type", &prtParticleType);
    prtTree->Branch("vx", &prtVx);
    prtTree->Branch("vy", &prtVy);
    prtTree->Branch("vz", &prtVz);
    prtTree->Branch("vt", &prtVt);
    prtTree->Branch("vr", &prtR);
    prtTree->Branch("px", &prtPx);
    prtTree->Branch("py", &prtPy);
    prtTree->Branch("pz", &prtPz);
    prtTree->Branch("AbsP", &prtAbsP);
    prtTree->Branch("m", &prtM);
    prtTree->Branch("q", &prtQ);
    prtTree->Branch("nhits", &prtNumHits);
    prtTree->Branch("ntracks", &prtNumTracks);
    prtTree->Branch("ntracks_majority", &prtNumTracksMajority);
  }

  const Acts::Logger& logger() const { return _logger; }

  void write(uint64_t eventId, const SimParticleContainer& particles,
             const HitParticlesMap& hitParticlesMap,
             const ProtoTrackContainer& seeds) {
    if (cfg.outputIsML) {
      return;
    }
    // compute the inverse mapping on-the-fly
    const auto& particleHitsMap = invertIndexMultimap(hitParticlesMap);
    // How often a particle was reconstructed.
    std::unordered_map<ActsFatras::Barcode, std::size_t> reconCount;
    reconCount.reserve(particles.size());
    // How often a particle was reconstructed as the majority particle.
    std::unordered_map<ActsFatras::Barcode, std::size_t> majorityCount;
    majorityCount.reserve(particles.size());

    // For each particle within a track, how many hits did it contribute
    std::vector<ParticleHitCount> particleHitCounts;

    // write per-seed performance measures
    {
      std::lock_guard<std::mutex> guardTrk(trkMutex);
      for (size_t itrack = 0; itrack < seeds.size(); ++itrack) {
        const auto& track = seeds[itrack];

        identifyContributingParticles(hitParticlesMap, track,
                                      particleHitCounts);
        // extract per-particle reconstruction counts
        // empty track hits counts could originate from a  buggy track finder
        // that results in empty tracks or from purely noise track where no hits
        // is from a particle.
        if (not particleHitCounts.empty()) {
          auto it = majorityCount
                        .try_emplace(particleHitCounts.front().particleId, 0u)
                        .first;
          it->second += 1;
        }
        for (const auto& hc : particleHitCounts) {
          auto it = reconCount.try_emplace(hc.particleId, 0u).first;
          it->second += 1;
        }

        trkEventId = eventId;
        trkTrackId = itrack;
        trkNumHits = track.size();
        trkNumParticles = particleHitCounts.size();
        trkParticleId.clear();
        trkParticleNumHitsTotal.clear();
        trkParticleNumHitsOnTrack.clear();
        for (const auto& phc : particleHitCounts) {
          trkParticleId.push_back(phc.particleId.value());
          // count total number of hits for this particle
          auto trueParticleHits =
              makeRange(particleHitsMap.equal_range(phc.particleId.value()));
          trkParticleNumHitsTotal.push_back(trueParticleHits.size());
          trkParticleNumHitsOnTrack.push_back(phc.hitCount);
        }

        trkTree->Fill();
      }
    }

    // write per-particle performance measures
    {
      std::lock_guard<std::mutex> guardPrt(trkMutex);
      for (const auto& particle : particles) {
        /*const auto prtPointer = particles.find(particle);
        if (!prtFindable(*prtPointer)) {
          continue;
        }*/
        // find all hits for this particle
        auto hits =
            makeRange(particleHitsMap.equal_range(particle.particleId()));
        // identification
        prtEventId = eventId;
        prtParticleId = particle.particleId().value();
        prtParticleType = particle.pdg();
        // kinematics
        prtVx = particle.position().x() / Acts::UnitConstants::mm;
        prtVy = particle.position().y() / Acts::UnitConstants::mm;
        prtVz = particle.position().z() / Acts::UnitConstants::mm;
        prtR = std::sqrt(prtVx * prtVx + prtVy * prtVy);
        prtVt = particle.time() / Acts::UnitConstants::ns;
        const auto p = particle.absMomentum() / Acts::UnitConstants::GeV;
        prtAbsP = p;
        prtPx = p * particle.unitDirection().x();
        prtPy = p * particle.unitDirection().y();
        prtPz = p * particle.unitDirection().z();
        prtM = particle.mass() / Acts::UnitConstants::GeV;
        prtQ = particle.charge() / Acts::UnitConstants::e;
        // reconstruction
        prtNumHits = hits.size();
        auto nt = reconCount.find(particle.particleId());
        prtNumTracks = (nt != reconCount.end()) ? nt->second : 0u;
        auto nm = majorityCount.find(particle.particleId());
        prtNumTracksMajority = (nm != majorityCount.end()) ? nm->second : 0u;

        prtTree->Fill();
      }
    }
  }
  /// Write everything to disk and close the file.
  void close() {
    if (cfg.outputIsML) {
      return;
    }
    if (not file) {
      ACTS_ERROR("Output file is not available");
      return;
    }
    file->Write();
    file->Close();
  }
};

FW::TrackSeedingPerformanceWriter::TrackSeedingPerformanceWriter(
    FW::TrackSeedingPerformanceWriter::Config cfg, Acts::Logging::Level lvl)
    : WriterT(cfg.inputSeeds, "TrackSeedingPerformanceWriter", lvl),
      m_impl(std::make_unique<Impl>(std::move(cfg), logger())),
      m_fakeRatePlotTool(m_impl->cfg.fakeRatePlotConfig, lvl),
      m_effPlotTool(m_impl->cfg.effPlotToolConfig, lvl),
      m_trackSummaryPlotTool(m_cfg.trackSummaryPlotToolConfig, lvl) {
  // Input track and truth collection name
  if (m_impl->cfg.inputSeeds.empty()) {
    throw std::invalid_argument("Missing input seeds collection");
  }
  if (m_impl->cfg.inputParticles.empty()) {
    throw std::invalid_argument("Missing input particles collection");
  }
  if (m_impl->cfg.outputFilename.empty()) {
    throw std::invalid_argument("Missing output filename");
  }

  if (m_impl->cfg.outputIsML) {
    return;
  }
  // the output file can not be given externally since TFile accesses to the
  // same file from multiple threads are unsafe.
  // must always be opened internally
  auto path = joinPaths(m_impl->cfg.outputDir, m_impl->cfg.outputFilename);
  m_outputFile = TFile::Open(path.c_str(), "RECREATE");
  if (not m_outputFile) {
    throw std::invalid_argument("Could not open '" + path + "'");
  }

  // initialize the efficiency plots tool
  m_effPlotTool.book(m_effPlotCache);
  m_fakeRatePlotTool.book(m_fakeRatePlotCache);
  m_trackSummaryPlotTool.book(m_trackSummaryPlotCache);
}

FW::TrackSeedingPerformanceWriter::~TrackSeedingPerformanceWriter() {
  if (m_impl->cfg.outputIsML) {
    return;
  }
  m_effPlotTool.clear(m_effPlotCache);
  m_fakeRatePlotTool.clear(m_fakeRatePlotCache);
  m_trackSummaryPlotTool.clear(m_trackSummaryPlotCache);

  if (m_outputFile) {
    m_outputFile->Close();
  }
}

FW::ProcessCode FW::TrackSeedingPerformanceWriter::endRun() {
  if (m_impl->cfg.outputIsML) {
    return ProcessCode::SUCCESS;
  }
  if (m_outputFile) {
    m_outputFile->cd();
    m_effPlotTool.write(m_effPlotCache);
    m_fakeRatePlotTool.write(m_fakeRatePlotCache);
    m_trackSummaryPlotTool.write(m_trackSummaryPlotCache);

    ACTS_INFO("Wrote performance plots to '" << m_outputFile->GetPath() << "'");
  }
  m_impl->close();
  return ProcessCode::SUCCESS;
}

void FW::TrackSeedingPerformanceWriter::printSeed(
    const Acts::Seed<SpacePoint>* seed) const {
  const SpacePoint* sp = seed->sp()[0];
  std::cout << "Sp Id # " << sp->Id() << ": # of particles "
            << sp->particles.size() << ": ";
  std::cout << sp->m_geoId.layer() << " (" << sp->x() << ", " << sp->y() << ", "
            << sp->z()
            << ") particle = " << sp->particles[0].particleId.particle()
            << ", barcode: " << sp->particles[0].particleId;
  sp = seed->sp()[1];
  std::cout << "; Sp Id # " << sp->Id() << ": # of particles "
            << sp->particles.size() << ": ";
  std::cout << sp->m_geoId.layer() << " (" << sp->x() << ", " << sp->y() << ", "
            << sp->z()
            << ") particle = " << sp->particles[0].particleId.particle()
            << ", barcode: " << sp->particles[0].particleId;
  sp = seed->sp()[2];
  std::cout << "; Sp Id # " << sp->Id() << ": # of particles "
            << sp->particles.size() << ": ";
  std::cout << sp->m_geoId.layer() << " (" << sp->x() << ", " << sp->y() << ", "
            << sp->z()
            << ") particle = " << sp->particles[0].particleId.particle()
            << ", barcode: " << sp->particles[0].particleId << " with count "
            << sp->particles[0].hitCount;
  std::cout << std::endl << std::endl;
}

bool FW::TrackSeedingPerformanceWriter::prtFindable(
    const ActsFatras::Particle& prt, const auto& particleHitsMap,
    const FW::GeometryIdMultimap<Acts::PlanarModuleCluster>& clusters) const {
  auto within = [](double x, double min, double max) {
    return (min <= x) and (x < max);
  };
  auto isValidparticle = [&](const auto& p) {
    const auto eta = Acts::VectorHelpers::eta(p.unitDirection());
    const auto phi = Acts::VectorHelpers::phi(p.unitDirection());
    const auto rho = Acts::VectorHelpers::perp(p.position());
    // find the corresponding hits for this particle
    const auto& hits = makeRange(particleHitsMap.equal_range(p.particleId()));
    // number of recorded hits
    size_t nHits = hits.size();
    return within(rho, 0., m_impl->cfg.rhoMax) and
           within(std::abs(p.position().z()), 0., m_impl->cfg.absZMax) and
           within(std::abs(eta), m_impl->cfg.absEtaMin,
                  m_impl->cfg.absEtaMax) and
           within(eta, m_impl->cfg.etaMin, m_impl->cfg.etaMax) and
           within(phi, m_impl->cfg.phiMin, m_impl->cfg.phiMax) and
           within(p.transverseMomentum(), m_impl->cfg.ptMin,
                  m_impl->cfg.ptMax) and
           within(nHits, m_impl->cfg.nHitsMin, m_impl->cfg.nHitsMax) and
           (m_impl->cfg.keepNeutral or (p.charge() != 0));
  };

  auto prtHits = makeRange(particleHitsMap.equal_range(prt.particleId()));
  bool hasOuterHit = false;
  size_t innerHitCount = 0;
  bool hasGreenHit = false;

  for (const auto& prtHit : prtHits) {
    size_t hit_id = prtHit.second;
    const auto& entry = clusters.begin() + hit_id;  // hit_id is 0-indexed
    if (entry == clusters.end()) {
      ACTS_INFO("Unable to find hit " << hit_id
                                      << ". (From prtFindable function)")
    }
    Acts::GeometryID geoId = entry->first;
    size_t volumeId = geoId.volume();
    if (12 <= volumeId && volumeId <= 14) {
      hasOuterHit = true;
    } else if (16 <= volumeId && volumeId <= 18) {
      hasGreenHit = true;
    } else if (volumeId == 8) {
      innerHitCount++;
    }
  }
  bool has3InnerHit = innerHitCount >= 3;
  return has3InnerHit && hasGreenHit && hasOuterHit && isValidparticle(prt);
}

std::set<ActsFatras::Barcode>
FW::TrackSeedingPerformanceWriter::identifySharedParticles(
    const Acts::Seed<SpacePoint>* seed) const {
  const SpacePoint* sp0 = seed->sp()[0];
  const SpacePoint* sp1 = seed->sp()[1];
  const SpacePoint* sp2 = seed->sp()[2];
  std::set<ActsFatras::Barcode> particles0;
  std::set<ActsFatras::Barcode> particles1;
  std::set<ActsFatras::Barcode> particles2;
  for (size_t i = 0; i < sp0->particles.size(); i++) {
    particles0.insert(sp0->particles[i].particleId);  // insert particle barcode
  }
  for (size_t i = 0; i < sp1->particles.size(); i++) {
    particles1.insert(sp1->particles[i].particleId);
  }
  for (size_t i = 0; i < sp2->particles.size(); i++) {
    particles2.insert(sp2->particles[i].particleId);
  }
  std::set<ActsFatras::Barcode> tmp;
  set_intersection(particles0.begin(), particles0.end(), particles1.begin(),
                   particles1.end(), std::inserter(tmp, tmp.end()));
  std::set<ActsFatras::Barcode> prtsInCommon;
  set_intersection(particles2.begin(), particles2.end(), tmp.begin(), tmp.end(),
                   std::inserter(prtsInCommon, prtsInCommon.end()));
  return prtsInCommon;
}

std::set<ActsFatras::Barcode> FW::TrackSeedingPerformanceWriter::analyzeSeed(
    const Acts::Seed<SpacePoint>* seed, const HitParticlesMap& hitParticlesMap,
    std::unordered_map<ActsFatras::Barcode, std::size_t>& truthCount,
    std::unordered_map<ActsFatras::Barcode, std::size_t>& fakeCount) const {
  std::set<ActsFatras::Barcode> prtsInCommon = identifySharedParticles(seed);
  if (prtsInCommon.size() > 0) {
    for (const auto& prt : prtsInCommon) {
      auto it = truthCount.try_emplace(prt, 0u).first;
      it->second += 1;
    }
  } else {
    std::vector<FW::ParticleHitCount> particleHitCounts;
    identifyContributingParticles(hitParticlesMap, seedToProtoTrack(seed),
                                  particleHitCounts);
    for (const auto hc : particleHitCounts) {
      auto it = fakeCount.try_emplace(hc.particleId, 0u).first;
      it->second += 1;
    }
  }
  return prtsInCommon;
}

ProtoTrack FW::TrackSeedingPerformanceWriter::seedToProtoTrack(
    const Acts::Seed<SpacePoint>* seed) const {
  ProtoTrack track;
  track.reserve(seed->sp().size());
  for (std::size_t i = 0; i < seed->sp().size(); i++) {
    track.emplace_back(seed->sp()[i]->Id());
  }
  return track;
}

void FW::TrackSeedingPerformanceWriter::writePlots(
    const std::vector<std::vector<Acts::Seed<SpacePoint>>>& seedVector,
    const IndexMultimap<ActsFatras::Barcode>& hitParticlesMap,
    const SimParticleContainer& particles,
    const FW::GeometryIdMultimap<Acts::PlanarModuleCluster>& clusters) {
  const auto& particleHitsMap = invertIndexMultimap(hitParticlesMap);
  // Exclusive access to the tree while writing
  std::lock_guard<std::mutex> lock(m_writeMutex);
  size_t nSeeds = 0;
  // Map from particles to how many times they were successfully found by a seed
  std::unordered_map<ActsFatras::Barcode, std::size_t> truthCount;
  // Map from particles to how many times they were involved in a seed that is
  // fake
  std::unordered_map<ActsFatras::Barcode, std::size_t> fakeCount;
  std::vector<SimMultiTrajectory> parameters;
  parameters.reserve(particles.size());
  for (auto& regionVec : seedVector) {
    nSeeds += regionVec.size();
    for (size_t i = 0; i < regionVec.size(); i++) {
      const Acts::Seed<SpacePoint>* seed = &regionVec[i];
      std::set<ActsFatras::Barcode> prtsInCommon =
          analyzeSeed(seed, hitParticlesMap, truthCount, fakeCount);
    }
  }
  size_t numPtAbove25 = 0;
  if (!m_impl->cfg.outputIsML) {
    // Fill the Efficiency and fake rate plots
    for (const auto& particle : particles) {
      if (particle.absMomentum() > 10) {
        numPtAbove25++;
      }
      if (!prtFindable(particle, particleHitsMap, clusters)) {
        continue;
      }
      const auto it1 = truthCount.find(particle.particleId());
      const auto it2 = fakeCount.find(particle.particleId());
      size_t nTruthMatchedSeeds = (it1 != truthCount.end()) ? it1->second : 0u;
      size_t nFakeSeeds = (it2 != fakeCount.end()) ? it2->second : 0u;
      m_fakeRatePlotTool.fill(m_fakeRatePlotCache, particle, nTruthMatchedSeeds,
                              nFakeSeeds);
      /// TODO: add the rest of the fake rate plots
      m_effPlotTool.fill(m_effPlotCache, particle, nTruthMatchedSeeds > 0);
    }
  }
  size_t nTrueSeeds = 0;       // true seed means it contains a particle
  size_t nDuplicateSeeds = 0;  // Number of seeds that re-find a particle
  size_t foundParticles = 0;
  // Number of particles we expect the seed finder to find
  size_t qualityCutPrts = 0;
  // number of already found particles we expect the seed finder to find
  size_t qualityCutFoundPrts = 0;
  for (const auto& tc : truthCount) {
    if (tc.second > 0) {
      foundParticles++;
    }
    const auto prtPointer = particles.find(tc.first);
    if (prtFindable(*prtPointer, particleHitsMap, clusters)) {
      qualityCutFoundPrts++;
    }
    nTrueSeeds += tc.second;
    if (tc.second > 1) {
      // -1 since duplicate seeds don't include first true seed
      nDuplicateSeeds += tc.second - 1;
    }
  }
  for (const auto& particle : particles) {
    if (prtFindable(particle, particleHitsMap, clusters)) {
      qualityCutPrts++;
    }
  }
  size_t nParticles = particles.size();
  if (m_impl->cfg.outputIsML) {
    std::cout << m_impl->cfg.mlTag << "seeds" << nSeeds << std::endl;
    std::cout << m_impl->cfg.mlTag << "eff"
              << (float)(10000 * qualityCutFoundPrts / (qualityCutPrts)) / 100.
              << std::endl;
    std::cout << m_impl->cfg.mlTag << "true" << nTrueSeeds << std::endl;
  } else {
    ACTS_INFO("Number of seeds generated: " << nSeeds)
    ACTS_INFO("Number of true seeds generated: " << nTrueSeeds)

    ACTS_INFO("Number of duplicate seeds generated: " << nDuplicateSeeds)
    ACTS_INFO(
        "Technical Efficiency (nTrueSeeds - nDuplicateSeeds / nSeeds) --- "
        << 100 * (nTrueSeeds - nDuplicateSeeds) / nSeeds << "%")
    ACTS_INFO("Raw Efficiency: (particles matched to truth) / nParticles = "
              << foundParticles << " / " << nParticles << " = "
              << 100 * foundParticles / nParticles << "%")
    ACTS_INFO("Fake rate (nSeeds - nTrueSeeds) / nSeeds --- "
              << (100 * (nSeeds - nTrueSeeds)) / nSeeds << "%")
    ACTS_INFO("Duplicate rate (nDuplicateSeeds / nSeeds) --- "
              << (100 * nDuplicateSeeds) / nSeeds << "%")
    ACTS_INFO("Efficiency "
              << qualityCutFoundPrts << "/" << qualityCutPrts << " = "
              << 100 * qualityCutFoundPrts / (qualityCutPrts) << "%")
    ACTS_DEBUG("Number of particles with absP > 25 = " << numPtAbove25)
  }
}

FW::ProcessCode FW::TrackSeedingPerformanceWriter::writeT(
    const AlgorithmContext& ctx,
    const std::vector<std::vector<Acts::Seed<SpacePoint>>>& seedVector) {
  // Read truth particles from input collection
  const auto& particles =
      ctx.eventStore.get<SimParticleContainer>(m_impl->cfg.inputParticles);
  // Read in clusters to be used for efficiency calculation
  const auto& clusters =
      ctx.eventStore.get<FW::GeometryIdMultimap<Acts::PlanarModuleCluster>>(
          m_impl->cfg.inputClusters);
  // Read in hit to particles map
  const auto& hitParticlesMap =
      ctx.eventStore.get<HitParticlesMap>(m_impl->cfg.inputHitParticlesMap);
  const auto& protoSeeds =
      ctx.eventStore.get<ProtoTrackContainer>(m_impl->cfg.inputProtoSeeds);
  // Write fake rate and efficiency plots
  if (protoSeeds.size() == 0) {
    ACTS_DEBUG("ProtoSeeds read in are empty")
    if (m_impl->cfg.outputIsML) {
      std::cout << m_impl->cfg.mlTag << "seeds" << 0 << std::endl;
      std::cout << m_impl->cfg.mlTag << "eff" << 0 << std::endl;
      std::cout << m_impl->cfg.mlTag << "true" << 0 << std::endl;
    }
  } else {
    writePlots(seedVector, hitParticlesMap, particles, clusters);
    // Write TTrees for histograms
    if (!m_cfg.outputIsML) {
      // Read seeds in as proto tracks
      m_impl->write(ctx.eventNumber, particles, hitParticlesMap, protoSeeds);
      ACTS_INFO("Wrote seed finder performance trees to "
                << m_impl->file->GetPath())
    }
  }
  return ProcessCode::SUCCESS;
}