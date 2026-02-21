#pragma once

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "nav3d_msgs/msg/route.hpp"
#include "nav3d_util/geometry_utils.hpp"
#include "nav3d_util/robot_utils.hpp"
#include "nav_msgs/msg/goals.hpp"

namespace nav3d_behavior_tree
{

class ExtractRouteNodesAsGoals : public BT::ActionNodeBase
{
public:
  ExtractRouteNodesAsGoals(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav3d_msgs::msg::Route>("route", "Route to extract nodes from"),
      BT::OutputPort<nav_msgs::msg::Goals>("goals", "Output goals for navigation")
    };
  }

private:
  void halt() override {}
  BT::NodeStatus tick() override;
};

}  // namespace nav3d_behavior_tree