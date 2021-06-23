// This file is part of the Acts project.
//
// Copyright (C) 2020 CERN for the benefit of the Acts project
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#pragma once

#include "ACTFW/EventData/GeometryContainers.hpp"
#include "ACTFW/EventData/IndexContainers.hpp"
#include "ACTFW/Framework/BareAlgorithm.hpp"
#include "Acts/Plugins/Digitization/PlanarModuleCluster.hpp"
#include "Acts/Seeding/InternalSeed.hpp"
#include "Acts/Seeding/InternalSpacePoint.hpp"
#include "Acts/Seeding/Seedfinder.hpp"
#include "Acts/Seeding/SpacePoint.hpp"
#include "ActsFatras/EventData/Barcode.hpp"

#include <set>

namespace FW {

/// An algorithm that tests the performance of seeding algorithms
class TestSeedAlgorithm : public FW::BareAlgorithm {
 public:
  struct Config {
    // currently not using outputHitIds.
    std::string outputHitIds;
    // input Clusters from the event#-hits.csv file.
    std::string inputClusters;
    // input particles for creating proto seeds
    std::string inputParticles;
    // input dir containing hits and truth information
    std::string inputDir;
    // used to get truth information into seeds about what particles are in what
    // space point.
    std::string inputHitParticlesMap;
    /// Which simulated (truth) hits collection to use. Not used currently.
    std::string inputSimulatedHits;
    // output seeds found by seeding algorithm
    std::string outputSeeds;
    // output seeds in proto track form
    std::string outputProtoSeeds;
    // Read in seed finder config
    Acts::SeedfinderConfig<SpacePoint> seedFinderCfg;
    // Controls whether to print out ML friendly output
    bool outputIsML = false;
    // potentially has input space points from csv readers
    std::string inputSpacePoints;
    //  controls whether we are given space points or clusters
    bool inputIsSPs = true;
  };

  TestSeedAlgorithm(const Config& cfg, Acts::Logging::Level level);

  /// Technically, space points can have multiple particles that are a part of
  /// them, so analyzeSeed finds how many particles are in common.
  /// @param seed The seed to be processed.
  /// @param particlesFoundBySeeds The set of particle barcodes already found.
  /// @param nDuplicateSeeds Number of seeds that find a particle already
  /// identified by a previous seed.
  ///
  /// Returns the number particles that are a part
  /// of all 3 spacePoints in the seed. Returning 0 means it's a fake seed.
  std::size_t analyzeSeed(const Acts::Seed<SpacePoint>* seed,
                          std::set<ActsFatras::Barcode>& particlesFoundBySeeds,
                          std::size_t& nDuplicateSeeds) const;

  /// @param cluster The hit with local hit information
  /// Returns a space point with a particle barcode stored in .particles for
  /// each particle that made this space point.
  SpacePoint* readSP(std::size_t hit_id, const Acts::GeometryID geoId,
                     const Acts::PlanarModuleCluster& cluster,
                     const IndexMultimap<ActsFatras::Barcode>& hitParticlesMap,
                     const AlgorithmContext& ctx) const;

  /// @brief Converts a seed to a proto track of 3 hits
  FW::ProtoTrack seedToProtoTrack(const Acts::Seed<SpacePoint>* seed) const;

  void printSeed(const Acts::Seed<SpacePoint>* seed) const;

  /// The framework execute method
  /// @param ctx The Algorithm context for multithreading
  FW::ProcessCode execute(const AlgorithmContext& ctx) const final override;

 private:
  Config m_cfg;
};

}  // namespace FW
