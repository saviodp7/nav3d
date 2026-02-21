#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"

namespace nav3d_behavior_tree
{

class TimeExpiredCondition : public BT::ConditionNode
{
public:
  TimeExpiredCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  TimeExpiredCondition() = delete;

  BT::NodeStatus tick() override;

  void initialize();

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("seconds", 1.0, "Seconds")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Time start_;
  double period_;
};

}  // namespace nav3d_behavior_tree