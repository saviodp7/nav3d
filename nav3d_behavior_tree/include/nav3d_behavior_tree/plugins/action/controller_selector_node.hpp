#pragma once

#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace nav3d_behavior_tree
{

class ControllerSelector : public BT::SyncActionNode
{
public:
  ControllerSelector(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "default_controller",
        "the default controller to use if there is not any external topic message received."),
      BT::InputPort<std::string>(
        "topic_name",
        "controller_selector",
        "the input topic name to select the controller"),
      BT::OutputPort<std::string>(
        "selected_controller",
        "Selected controller by subscription")
    };
  }

private:
  void initialize();
  void createROSInterfaces();

  BT::NodeStatus tick() override;

  void callbackControllerSelect(const std_msgs::msg::String::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr controller_selector_sub_;

  std::string last_selected_controller_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  std::string topic_name_;
};

}  // namespace nav3d_behavior_tree