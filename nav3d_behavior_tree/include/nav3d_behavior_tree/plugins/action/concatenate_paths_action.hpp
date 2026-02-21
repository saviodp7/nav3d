#pragma once

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "nav_msgs/msg/path.hpp"

namespace nav3d_behavior_tree
{

class ConcatenatePaths : public BT::ActionNodeBase
{
public:
  ConcatenatePaths(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("input_path1", "Input Path 1 to cancatenate"),
      BT::InputPort<nav_msgs::msg::Path>("input_path2", "Input Path 2 to cancatenate"),
      BT::OutputPort<nav_msgs::msg::Path>("output_path", "Paths concatenated"),
    };
  }

private:
  void halt() override {}
  BT::NodeStatus tick() override;
};

}  // namespace nav3d_behavior_tree