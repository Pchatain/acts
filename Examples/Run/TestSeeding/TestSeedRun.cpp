// This file is part of the Acts project.
//
// Copyright (C) 2017-2019 CERN for the benefit of the Acts project
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "ACTFW/Digitization/HitSmearing.hpp"
#include "ACTFW/Fitting/FittingAlgorithm.hpp"
#include "ACTFW/Framework/Sequencer.hpp"
#include "ACTFW/Framework/WhiteBoard.hpp"
#include "ACTFW/GenericDetector/GenericDetector.hpp"
#include "ACTFW/Geometry/CommonGeometry.hpp"
#include "ACTFW/Io/Csv/CsvOptionsReader.hpp"
#include "ACTFW/Io/Csv/CsvOptionsWriter.hpp"
#include "ACTFW/Io/Csv/CsvParticleReader.hpp"
#include "ACTFW/Io/Csv/CsvPlanarClusterWriter.hpp"
#include "ACTFW/Io/Csv/CsvSpacePointReader.hpp"
#include "ACTFW/Io/Performance/TrackFinderPerformanceWriter.hpp"
#include "ACTFW/Io/Performance/TrackFitterPerformanceWriter.hpp"
#include "ACTFW/Io/Performance/TrackSeedingPerformanceWriter.hpp"
#include "ACTFW/Io/Root/RootTrajectoryWriter.hpp"
#include "ACTFW/Options/CommonOptions.hpp"
#include "ACTFW/Plugins/BField/BFieldOptions.hpp"
#include "ACTFW/TruthTracking/ParticleSmearing.hpp"
#include "ACTFW/TruthTracking/TruthSeedSelector.hpp"
#include "ACTFW/TruthTracking/TruthTrackFinder.hpp"
#include "ACTFW/Utilities/Options.hpp"
#include "ACTFW/Utilities/Paths.hpp"
#include "Acts/Seeding/Seedfinder.hpp"
#include <Acts/Utilities/Units.hpp>

#include <cstdlib>
#include <memory>

#include "SeedFinderOptions.hpp"
#include "TestSeedAlgorithm.hpp"

using namespace Acts::UnitLiterals;
using namespace FW;

int main(int argc, char* argv[]) {
  GenericDetector detector;

  // setup options
  // every component should have an associated option setup function
  // that should be called here.
  auto desc = Options::makeDefaultOptions();
  Options::addSequencerOptions(desc);
  Options::addInputOptions(desc);
  Options::addRandomNumbersOptions(desc);
  Options::addOutputOptions(desc);
  Options::addGeometryOptions(desc);
  Options::addMaterialOptions(desc);
  detector.addOptions(desc);
  Options::addBFieldOptions(desc);
  Options::addCsvWriterOptions(desc);
  Options::addSeedFinderOptions(desc);
  Options::addMLOutput(desc);
  Options::addSeedPerfOptions(desc);

  // parse options from command line flags
  auto vm = Options::parse(desc, argc, argv);
  // an empty varaibles map indicates an error
  if (vm.empty()) {
    return EXIT_FAILURE;
  }

  // setup the sequencer first w/ config derived from options
  Sequencer sequencer(FW::Options::readSequencerConfig(vm));

  // extract some common options
  auto logLevel = Options::readLogLevel(vm);
  auto inputDir = vm["input-dir"].as<std::string>();
  auto outputDir = ensureWritableDirectory(vm["output-dir"].as<std::string>());
  auto rnd =
      std::make_shared<FW::RandomNumbers>(Options::readRandomNumbersConfig(vm));
  // Setup detector geometry
  auto geometry = Geometry::build(vm, detector);
  auto trackingGeometry = geometry.first;
  // Add context decorators
  for (auto cdr : geometry.second) {
    sequencer.addContextDecorator(cdr);
  }
  // Setup the magnetic field
  auto magneticField = Options::readBField(vm);

  bool ITK = true;  // DECIDES WHETHER TO USE ITK DATASET FROM ATHENA AFTER
                    // MODIFICATIONS I MADE

  // Read particles (initial states) and clusters from CSV files
  auto particleReader = Options::readCsvParticleReaderConfig(vm);
  particleReader.inputStem = "particles_initial";
  particleReader.outputParticles = "particles_initial";
  sequencer.addReader(
      std::make_shared<CsvParticleReader>(particleReader, logLevel));
  // Read clusters from CSV files
  auto clusterReaderCfg = Options::readCsvPlanarClusterReaderConfig(vm);
  clusterReaderCfg.trackingGeometry = trackingGeometry;
  clusterReaderCfg.outputClusters = "clusters";
  clusterReaderCfg.outputHitIds = "hit_ids";
  clusterReaderCfg.outputHitParticlesMap = "hit_particles_map";
  clusterReaderCfg.outputSimulatedHits = "hits";
  FW::CsvSpacePointReader::Config spacePointReaderCfg;
  spacePointReaderCfg.inputDir = clusterReaderCfg.inputDir;
  spacePointReaderCfg.outputSpacePoints = "space_points";
  spacePointReaderCfg.outputClusters = "clusters";
  spacePointReaderCfg.outputHitIds = "hit_ids";
  spacePointReaderCfg.outputHitParticlesMap = "hit_particles_map";
  if (!ITK) {
    sequencer.addReader(
        std::make_shared<CsvPlanarClusterReader>(clusterReaderCfg, logLevel));
  } else {
    // std::cout << "We added right reader" << std::endl;
    sequencer.addReader(
        std::make_shared<CsvSpacePointReader>(spacePointReaderCfg, logLevel));
  }

  const auto& inputParticles = particleReader.outputParticles;

  // add Seeding Algorithm that finds the seeds
  bool outputIsML = Options::readMLOutputConfig(vm);
  Acts::SeedfinderConfig<SpacePoint> seedFinderCfg =
      Options::readSeedFinderConfig(vm);
  FW::TestSeedAlgorithm::Config testSeedCfg;
  testSeedCfg.outputIsML = outputIsML;
  testSeedCfg.seedFinderCfg = seedFinderCfg;
  testSeedCfg.inputHitParticlesMap = "hit_particles_map";
  // testSeedCfg.inputSimulatedHits = "hits";
  testSeedCfg.inputDir = inputDir;
  testSeedCfg.inputParticles = inputParticles;
  testSeedCfg.outputHitIds = "hit_ids";
  testSeedCfg.inputClusters = "clusters";
  testSeedCfg.outputSeeds = "output_seeds";
  testSeedCfg.outputProtoSeeds = "output_proto_seeds";
  if (ITK) {
    testSeedCfg.inputSpacePoints = spacePointReaderCfg.outputSpacePoints;
  }
  sequencer.addAlgorithm(
      std::make_shared<FW::TestSeedAlgorithm>(testSeedCfg, logLevel));

  //   FW::TrackSeedingPerformanceWriter::Config
  auto seedPerfCfg = Options::readSeedPerfConfig(vm);
  // std::cout << "minPt is " << seedPerfCfg.ptMin << std::endl;
  seedPerfCfg.outputIsML = outputIsML;
  seedPerfCfg.inputSeeds = testSeedCfg.outputSeeds;
  seedPerfCfg.inputProtoSeeds = testSeedCfg.outputProtoSeeds;
  seedPerfCfg.inputParticles = inputParticles;
  seedPerfCfg.inputClusters = testSeedCfg.inputClusters;
  seedPerfCfg.inputHitParticlesMap =
      clusterReaderCfg.outputHitParticlesMap;  // same as for spreader
  seedPerfCfg.outputDir = outputDir;
  sequencer.addWriter(
      std::make_shared<TrackSeedingPerformanceWriter>(seedPerfCfg, logLevel));

  // --------------------------------------------------------------------
  // Connect to track Finder                                            |
  // --------------------------------------------------------------------
  /*
    // Create smeared measurements
    HitSmearing::Config hitSmearingCfg;
    hitSmearingCfg.inputSimulatedHits = clusterReaderCfg.outputSimulatedHits;
    hitSmearingCfg.outputSourceLinks = "sourcelinks";
    hitSmearingCfg.sigmaLoc0 = 25_um;
    hitSmearingCfg.sigmaLoc1 = 100_um;
    hitSmearingCfg.randomNumbers = rnd;
    hitSmearingCfg.trackingGeometry = trackingGeometry;
    sequencer.addAlgorithm(
        std::make_shared<HitSmearing>(hitSmearingCfg, logLevel));
    // Create smeared particles states
    ParticleSmearing::Config particleSmearingCfg;
    particleSmearingCfg.inputParticles = inputParticles;
    particleSmearingCfg.outputTrackParameters = "smearedparameters";
    particleSmearingCfg.randomNumbers = rnd;
    // Gaussian sigmas to smear particle parameters
    particleSmearingCfg.sigmaD0 = 20_um;
    particleSmearingCfg.sigmaD0PtA = 30_um;
    particleSmearingCfg.sigmaD0PtB = 0.3 / 1_GeV;
    particleSmearingCfg.sigmaZ0 = 20_um;
    particleSmearingCfg.sigmaZ0PtA = 30_um;
    particleSmearingCfg.sigmaZ0PtB = 0.3 / 1_GeV;
    particleSmearingCfg.sigmaPhi = 1_degree;
    particleSmearingCfg.sigmaTheta = 1_degree;
    particleSmearingCfg.sigmaPRel = 0.01;
    particleSmearingCfg.sigmaT0 = 1_ns;
    sequencer.addAlgorithm(
        std::make_shared<ParticleSmearing>(particleSmearingCfg, logLevel));

    // setup the fitter
    FittingAlgorithm::Config fitter;
    fitter.inputSourceLinks = hitSmearingCfg.outputSourceLinks;
    fitter.inputProtoTracks = testSeedCfg.outputProtoSeeds;
    fitter.inputInitialTrackParameters =
        particleSmearingCfg.outputTrackParameters;
    fitter.outputTrajectories = "trajectories";
    fitter.fit = FittingAlgorithm::makeFitterFunction(trackingGeometry,
                                                      magneticField, logLevel);
    sequencer.addAlgorithm(std::make_shared<FittingAlgorithm>(fitter,
    logLevel));

    // write tracks from fitting
    RootTrajectoryWriter::Config trackWriter;
    trackWriter.inputParticles = inputParticles;
    trackWriter.inputTrajectories = fitter.outputTrajectories;
    trackWriter.outputDir = outputDir;
    trackWriter.outputFilename = "tracks.root";
    trackWriter.outputTreename = "tracks";
    sequencer.addWriter(
        std::make_shared<RootTrajectoryWriter>(trackWriter, logLevel));

    // write reconstruction performance data
    TrackFinderPerformanceWriter::Config perfFinder;
    perfFinder.inputParticles = inputParticles;
    perfFinder.inputHitParticlesMap = clusterReaderCfg.outputHitParticlesMap;
    perfFinder.inputProtoTracks = testSeedCfg.outputProtoSeeds;
    perfFinder.outputDir = outputDir;
    sequencer.addWriter(
        std::make_shared<TrackFinderPerformanceWriter>(perfFinder, logLevel));
    TrackFitterPerformanceWriter::Config perfFitter;
    perfFitter.inputParticles = inputParticles;
    perfFitter.inputTrajectories = fitter.outputTrajectories;
    perfFitter.outputDir = outputDir;
    sequencer.addWriter(
        std::make_shared<TrackFitterPerformanceWriter>(perfFitter, logLevel));
  */
  // Run all configured algorithms and return the appropriate status.
  return sequencer.run();
}
