#pragma once

#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace nav3d_behavior_tree
{

class GoalCheckerSelector : public BT::SyncActionNode
{
public:
  GoalCheckerSelector(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "default_goal_checker",
        "the default goal_checker to use if there is not any external topic message received."),
      BT::InputPort<std::string>(
        "topic_name",
        "goal_checker_selector",
        "the input topic name to select the goal_checker"),
      BT::OutputPort<std::string>(
        "selected_goal_checker",
        "Selected goal_checker by subscription")
    };
  }

private:
  void initialize();
  void createROSInterfaces();

  BT::NodeStatus tick() override;

  void callbackGoalCheckerSelect(const std_msgs::msg::String::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr goal_checker_selector_sub_;

  std::string last_selected_goal_checker_;

  rclcpp::Node::SharedPtr node_;

  std::string topic_name_;
};

}  // namespace nav3d_behavior_tree