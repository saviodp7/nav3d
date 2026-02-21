#pragma once

#include <limits>
#include <memory>
#include <string>

#include "behaviortree_cpp/decorator_node.h"
#include "behaviortree_cpp/json_export.h"
#include "nav3d_behavior_tree/bt_utils.hpp"
#include "nav3d_behavior_tree/json_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav3d_behavior_tree
{

class PathLongerOnApproach : public BT::DecoratorNode
{
public:
  PathLongerOnApproach(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    BT::RegisterJsonDefinition<nav_msgs::msg::Path>();

    return {
      BT::InputPort<nav_msgs::msg::Path>("path", "Planned Path"),
      BT::InputPort<double>(
        "prox_len", 3.0,
        "Proximity length (m) for the path to be longer on approach"),
      BT::InputPort<double>(
        "length_factor", 2.0,
        "Length multiplication factor to check if the path is significantly longer"),
    };
  }

  BT::NodeStatus tick() override;

private:
  bool isPathUpdated(
    nav_msgs::msg::Path & new_path,
    nav_msgs::msg::Path & old_path);

  bool isRobotInGoalProximity(
    nav_msgs::msg::Path & old_path,
    double & prox_leng);

  bool isNewPathLonger(
    nav_msgs::msg::Path & new_path,
    nav_msgs::msg::Path & old_path,
    double & length_factor);

private:
  nav_msgs::msg::Path new_path_;
  nav_msgs::msg::Path old_path_;
  double prox_len_ = std::numeric_limits<double>::max();
  double length_factor_ = std::numeric_limits<double>::max();
  rclcpp::Node::SharedPtr node_;
  bool first_time_ = true;
};

}  // namespace nav3d_behavior_tree