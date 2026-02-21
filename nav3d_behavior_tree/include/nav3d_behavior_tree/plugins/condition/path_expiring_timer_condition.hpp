#pragma once

#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_cpp/json_export.h"
#include "nav3d_behavior_tree/bt_utils.hpp"
#include "nav3d_behavior_tree/json_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav3d_behavior_tree
{

class PathExpiringTimerCondition : public BT::ConditionNode
{
public:
  PathExpiringTimerCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  PathExpiringTimerCondition() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    BT::RegisterJsonDefinition<nav_msgs::msg::Path>();

    return {
      BT::InputPort<double>("seconds", 1.0, "Seconds"),
      BT::InputPort<nav_msgs::msg::Path>("path")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Time start_;
  nav_msgs::msg::Path prev_path_;
  double period_;
  bool first_time_;
};

}  // namespace nav3d_behavior_tree