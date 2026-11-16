#include <algorithm>
#include <sstream>
#include <boost/property_tree/info_parser.hpp>
#include <ocs2_core/misc/LoadStdVectorOfPair.h>

int main(int argc, char** argv) {
  if (argc != 2) return 2;
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(argv[1], pt);
  const auto prefix = std::string("selfCollision.");
  std::vector<std::pair<std::string, std::string>> links;
  std::vector<std::pair<size_t, size_t>> objects;
  ocs2::loadData::loadStdVectorOfPair(argv[1], prefix + "collisionLinkPairs", links, false);
  ocs2::loadData::loadStdVectorOfPair(argv[1], prefix + "collisionObjectPairs", objects, false);
  std::cout << pt.get<bool>(prefix + "activate") << ' '
            << pt.get<double>(prefix + "minimumDistance") << ' '
            << pt.get<double>(prefix + "mu") << ' '
            << pt.get<double>(prefix + "delta") << '\n';
  std::cout << links.size() << ' ' << objects.size() << '\n';
  for (const auto& p : links) std::cout << p.first << ',' << p.second << '\n';
  std::cout << "OBJECTS\n";
  for (const auto& p : objects) std::cout << p.first << ',' << p.second << '\n';
}
