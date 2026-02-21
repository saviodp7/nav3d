#pragma once

#include <string>
#include <vector>

#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_cpp/json_export.h"
#include "nav3d_behavior_tree/bt_utils.hpp"
#include "nav3d_behavior_tree/json_utils.hpp"
#include "nav_msgs/msg/goals.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav3d_behavior_tree
{

class GloballyUpdatedGoalCondition : public BT::ConditionNode
{
public:
  GloballyUpdatedGoalCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  GloballyUpdatedGoalCondition() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    BT::RegisterJsonDefinition<geometry_msgs::msg::PoseStamped>();
    BT::RegisterJsonDefinition<nav_msgs::msg::Goals>();

    return {
      BT::InputPort<nav_msgs::msg::Goals>(
        "goals", "Vector of navigation goals"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "goal", "Navigation goal"),
    };
  }

private:
  bool first_time;
  rclcpp::Node::SharedPtr node_;
  geometry_msgs::msg::PoseStamped goal_;
  nav_msgs::msg::Goals goals_;
};

}  // namespace nav3d_behavior_tree