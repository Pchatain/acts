add_executable(
  ActsExampleTestSeedAlgorithm
  TestSeedRun.cpp
  TestSeedAlgorithm.cpp
  SeedFinderOptions.cpp)
target_link_libraries(
  ActsExampleTestSeedAlgorithm
  PRIVATE
    ActsCore
    ActsExamplesFramework 
    ActsExamplesCommon 
    ActsExamplesDigitization
    ActsExamplesFitting
    ActsExamplesDetectorGeneric
    ActsExamplesMagneticField
    ActsExamplesTruthTracking
    ActsExamplesIoCsv
    ActsExamplesIoPerformance
    Boost::program_options)

install(
  TARGETS ActsExampleTestSeedAlgorithm
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})