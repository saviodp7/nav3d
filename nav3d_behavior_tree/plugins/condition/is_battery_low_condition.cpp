#include <string>
#include <functional>

#include "nav3d_behavior_tree/plugins/condition/is_battery_low_condition.hpp"

namespace nav3d_behavior_tree
{

IsBatteryLowCondition::IsBatteryLowCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  battery_topic_("/battery_status"),
  min_battery_(0.0),
  is_voltage_(false),
  is_battery_low_(false)
{
  initialize();

  callback_group_executor_.spin_some(std::chrono::nanoseconds(1));
}

void IsBatteryLowCondition::initialize()
{
  getInput("min_battery", min_battery_);
  getInput("is_voltage", is_voltage_);

  createROSInterfaces();
}

void IsBatteryLowCondition::createROSInterfaces()
{
  std::string battery_topic_new;
  getInput("battery_topic", battery_topic_new);

  if (battery_topic_new != battery_topic_ || !battery_sub_) {
    battery_topic_ = battery_topic_new;
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive,
      false);
    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = callback_group_;
    battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
      battery_topic_,
      rclcpp::SystemDefaultsQoS(),
      std::bind(&IsBatteryLowCondition::batteryCallback, this, std::placeholders::_1),
      sub_option);
  }
}

BT::NodeStatus IsBatteryLowCondition::tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
  }

  callback_group_executor_.spin_some();
  return is_battery_low_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void IsBatteryLowCondition::batteryCallback(sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  if (is_voltage_) {
    is_battery_low_ = msg->voltage <= min_battery_;
  } else {
    is_battery_low_ = msg->percentage <= min_battery_;
  }
}

}  // namespace nav3d_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav3d_behavior_tree::IsBatteryLowCondition>("IsBatteryLow");
}