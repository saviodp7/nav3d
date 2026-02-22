#pragma once

#include <chrono>
#include <memory>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

#include "nav3d_msgs/srv/is_path_valid.hpp"

namespace nav3d_behavior_tree
{

class IsPathValidCondition : public BT::ConditionNode
{
public:
  IsPathValidCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path"),
      BT::InputPort<std::string>("service_name", "is_path_valid"),
      BT::InputPort<std::chrono::milliseconds>("server_timeout")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<nav3d_msgs::srv::IsPathValid>::SharedPtr client_;
  std::string service_name_{"is_path_valid"};
  std::chrono::milliseconds server_timeout_{1000};
};

}  // namespace nav3d_behavior_tree