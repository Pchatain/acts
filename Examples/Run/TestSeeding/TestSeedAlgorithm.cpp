// This file is part of the Acts project.
//
// Copyright (C) 2020 CERN for the benefit of the Acts project
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "TestSeedAlgorithm.hpp"

#include "ACTFW/EventData/IndexContainers.hpp"
#include "ACTFW/EventData/SimHit.hpp"
#include "ACTFW/EventData/SimIdentifier.hpp"
#include "ACTFW/EventData/SimParticle.hpp"
#include "ACTFW/EventData/SimVertex.hpp"
#include "ACTFW/Framework/WhiteBoard.hpp"
#include "ACTFW/Io/Csv/CsvPlanarClusterReader.hpp"
#include "Acts/Plugins/Digitization/PlanarModuleCluster.hpp"
#include "Acts/Seeding/ATLASCuts.hpp"
#include "Acts/Seeding/BinFinder.hpp"
#include "Acts/Seeding/BinnedSPGroup.hpp"
#include "Acts/Seeding/InternalSeed.hpp"
#include "Acts/Seeding/InternalSpacePoint.hpp"
#include "Acts/Seeding/Seed.hpp"
#include "Acts/Seeding/SeedFilter.hpp"
#include "Acts/Seeding/Seedfinder.hpp"
//#include "Acts/Seeding/SpacePoint.hpp"
#include "Acts/Seeding/SpacePointGrid.hpp"
#include "Acts/Utilities/Logger.hpp"
#include "Acts/Utilities/Units.hpp"

#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <boost/type_erasure/any_cast.hpp>

FW::TestSeedAlgorithm::TestSeedAlgorithm(
    const FW::TestSeedAlgorithm::Config& cfg, Acts::Logging::Level level)
    : FW::BareAlgorithm("TestSeedAlgorithm", level), m_cfg(std::move(cfg)) {
  // Is inputClusters already checked by base constructor?
  // I think this is only true for the writer
  if (m_cfg.inputClusters.empty()) {
    throw std::invalid_argument(
        "Missing clusters input collection with the hits");
  }
}

SpacePoint* FW::TestSeedAlgorithm::readSP(
    std::vector<const SpacePoint*>& spVec, const Acts::GeometryID geoId,
    const Acts::PlanarModuleCluster& cluster,
    const AlgorithmContext& ctx) const {
  const auto& parameters = cluster.parameters();
  Acts::Vector2D localPos(parameters[0], parameters[1]);
  Acts::Vector3D globalFakeMom(1, 1, 1);
  Acts::Vector3D globalPos(0, 0, 0);
  // transform local into global position information
  cluster.referenceObject().localToGlobal(ctx.geoContext, localPos,
                                          globalFakeMom, globalPos);
  float x, y, z, r, varianceR, varianceZ;
  x = globalPos.x();
  y = globalPos.y();
  z = globalPos.z();
  r = std::sqrt(x * x + y * y);
  varianceR = 0;  // initialized to 0 becuse they don't affect seeds generated
  varianceZ = 0;  // initialized to 0 becuse they don't affect seeds generated

  SpacePoint* sp = new SpacePoint{x, y, z, r, -1, varianceR, varianceZ};
  spVec.push_back(sp);
  return sp;
}

FW::ProcessCode FW::TestSeedAlgorithm::execute(
    const AlgorithmContext& ctx /*,
    const FW::GeometryIdMultimap<Acts::PlanarModuleCluster>& clusters*/) const {
  // read in the hits
  const auto& clusters =
      ctx.eventStore.get<FW::GeometryIdMultimap<Acts::PlanarModuleCluster>>(
          m_cfg.inputClusters);

  // create the space points
  std::size_t clustCounter = 0;
  std::size_t numIgnored = 0;
  std::vector<const SpacePoint*> spVec;
  for (const auto& entry : clusters) {
    Acts::GeometryID geoId = entry.first;
    const Acts::PlanarModuleCluster& cluster = entry.second;
    std::size_t volnum = geoId.volume();
    std::size_t laynum = geoId.layer();
    if ((clustCounter + numIgnored) % 1000 == 0) {
      ACTS_INFO("clust " << clustCounter + numIgnored
                         << " has layer() = " << geoId.layer()
                         << " and volume() = " << geoId.volume()
                         << " and value() " << geoId.value() << " and boundary "
                         << geoId.boundary() << " and approach "
                         << geoId.approach())
    }
    if (volnum == 7 || volnum == 8 || volnum == 9) {
      if (laynum == 2 || laynum == 4 || laynum == 6) {
        SpacePoint* sp = readSP(spVec, geoId, cluster, ctx);
        clustCounter++;
      } else {
        numIgnored++;
      }
    } else {
      numIgnored++;
    }
  }

  Acts::SeedfinderConfig<SpacePoint> config;
  // silicon detector max
  config.rMax = 160.;
  config.deltaRMin = 5.;
  config.deltaRMax = 160.;
  config.collisionRegionMin = -250.;
  config.collisionRegionMax = 250.;
  config.zMin = -2800.;
  config.zMax = 2800.;
  config.maxSeedsPerSpM = 5;
  // 2.7 eta
  config.cotThetaMax = 7.40627;
  config.sigmaScattering = 1.00000;

  config.minPt = 500.;
  config.bFieldInZ = 0.00199724;

  config.beamPos = {-.5, -.5};
  config.impactMax = 10.;

  auto bottomBinFinder = std::make_shared<Acts::BinFinder<SpacePoint>>(
      Acts::BinFinder<SpacePoint>());
  auto topBinFinder = std::make_shared<Acts::BinFinder<SpacePoint>>(
      Acts::BinFinder<SpacePoint>());
  Acts::SeedFilterConfig sfconf;
  Acts::ATLASCuts<SpacePoint> atlasCuts = Acts::ATLASCuts<SpacePoint>();
  config.seedFilter = std::make_unique<Acts::SeedFilter<SpacePoint>>(
      Acts::SeedFilter<SpacePoint>(sfconf, &atlasCuts));
  Acts::Seedfinder<SpacePoint> a(config);

  // covariance tool, sets covariances per spacepoint as required
  auto ct = [=](const SpacePoint& sp, float, float, float) -> Acts::Vector2D {
    return {sp.varianceR, sp.varianceZ};
  };

  // setup spacepoint grid config
  Acts::SpacePointGridConfig gridConf;
  gridConf.bFieldInZ = config.bFieldInZ;
  gridConf.minPt = config.minPt;
  gridConf.rMax = config.rMax;
  gridConf.zMax = config.zMax;
  gridConf.zMin = config.zMin;
  gridConf.deltaRMax = config.deltaRMax;
  gridConf.cotThetaMax = config.cotThetaMax;

  // create grid with bin sizes according to the configured geometry
  std::unique_ptr<Acts::SpacePointGrid<SpacePoint>> grid =
      Acts::SpacePointGridCreator::createGrid<SpacePoint>(gridConf);
  auto spGroup = Acts::BinnedSPGroup<SpacePoint>(spVec.begin(), spVec.end(), ct,
                                                 bottomBinFinder, topBinFinder,
                                                 std::move(grid), config);

  std::vector<std::vector<Acts::Seed<SpacePoint>>> seedVector;
  auto start = std::chrono::system_clock::now();
  auto groupIt = spGroup.begin();
  auto endOfGroups = spGroup.end();

  // actually executues the seed finding algoirthm here
  for (; !(groupIt == endOfGroups); ++groupIt) {
    seedVector.push_back(a.createSeedsForGroup(
        groupIt.bottom(), groupIt.middle(), groupIt.top()));
  }
  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_seconds = end - start;
  std::cout << "time to create seeds: " << elapsed_seconds.count() << std::endl;
  std::cout << "Number of regions: " << seedVector.size() << std::endl;
  int numSeeds = 0;
  for (auto& outVec : seedVector) {
    numSeeds += outVec.size();
  }
  std::cout << "Number of seeds generated: " << numSeeds << std::endl;
  ACTS_INFO("Number of clusters (hits) used is: " << clustCounter)

  return FW::ProcessCode::SUCCESS;
}
