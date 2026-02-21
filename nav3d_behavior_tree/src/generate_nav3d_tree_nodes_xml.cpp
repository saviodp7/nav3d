#include <fstream>
#include <string>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_cpp/xml_parsing.h"

#include "plugins_list.hpp"
#include "nav3d_util/string_utils.hpp"

int main()
{
  BT::BehaviorTreeFactory factory;

  std::vector<std::string> plugins_list =
    nav3d_util::split(nav3d::details::BT_BUILTIN_PLUGINS, ';');

  for (const auto & plugin : plugins_list) {
    std::cout << "Loading: " << plugin << "\n";
    factory.registerFromPlugin(BT::SharedLibrary::getOSName(plugin));
  }

  std::cout << "\nGenerating file: nav3d_tree_nodes.xml\n"
            << "\nCompare it with the one in the git repo and update the latter if necessary.\n";

  std::ofstream xml_file("nav3d_tree_nodes.xml");
  xml_file << BT::writeTreeNodesModelXML(factory) << std::endl;

  return 0;
}