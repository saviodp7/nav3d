#pragma once

#include <memory>
#include <string>

#include "behaviortree_cpp/decorator_node.h"
#include "behaviortree_cpp/json_export.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav3d_behavior_tree/bt_utils.hpp"
#include "nav3d_behavior_tree/json_utils.hpp"
#include "nav_msgs/msg/goals.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav3d_behavior_tree
{

class GoalUpdater : public BT::DecoratorNode
{
public:
  GoalUpdater(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    BT::RegisterJsonDefinition<geometry_msgs::msg::PoseStamped>();
    BT::RegisterJsonDefinition<nav_msgs::msg::Goals>();

    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("input_goal", "Original Goal"),
      BT::InputPort<nav_msgs::msg::Goals>("input_goals", "Original Goals"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>(
        "output_goal",
        "Received Goal by subscription"),
      BT::OutputPort<nav_msgs::msg::Goals>(
        "output_goals",
        "Received Goals by subscription")
    };
  }

private:
  void initialize();
  void createROSInterfaces();

  BT::NodeStatus tick() override;

  void callback_updated_goal(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void callback_updated_goals(const nav_msgs::msg::Goals::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Goals>::SharedPtr goals_sub_;

  geometry_msgs::msg::PoseStamped last_goal_received_;
  bool last_goal_received_set_{false};
  nav_msgs::msg::Goals last_goals_received_;
  bool last_goals_received_set_{false};

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  std::string goal_updater_topic_;
  std::string goals_updater_topic_;
};

}  // namespace nav3d_behavior_tree