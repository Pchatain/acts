#include "Acts/Seeding/Seedfinder.hpp"
#include "Acts/Seeding/SpacePoint.hpp"
#include <Acts/Utilities/Logger.hpp>

#include <string>

#include <boost/program_options.hpp>

namespace FW {
namespace Options {

/// Add seed finder options such as min pT for seeds.
void addSeedFinderOptions(boost::program_options::options_description& opt);

/// Add an option to output ML friendly output
void addMLOutput(boost::program_options::options_description& opt);

/// Read the seed finder config.
Acts::SeedfinderConfig<SpacePoint> readSeedFinderConfig(
    const boost::program_options::variables_map& vm);

bool readMLOutputConfig(const boost::program_options::variables_map& vm);
}  // namespace Options
}  // namespace FW