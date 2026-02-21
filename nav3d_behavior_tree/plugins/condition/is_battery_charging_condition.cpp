#include <string>
#include <functional>

#include "nav3d_behavior_tree/plugins/condition/is_battery_charging_condition.hpp"

namespace nav3d_behavior_tree
{

IsBatteryChargingCondition::IsBatteryChargingCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  battery_topic_("/battery_status"),
  is_battery_charging_(false)
{
  initialize();

  callback_group_executor_.spin_some(std::chrono::nanoseconds(1));
}

void IsBatteryChargingCondition::initialize()
{
  createROSInterfaces();
}

void IsBatteryChargingCondition::createROSInterfaces()
{
  std::string battery_topic_new;
  getInput("battery_topic", battery_topic_new);

  if (battery_topic_new != battery_topic_ || !battery_sub_) {
    battery_topic_ = battery_topic_new;
    auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    callback_group_ = node->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive,
      false);
    callback_group_executor_.add_callback_group(callback_group_, node->get_node_base_interface());

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = callback_group_;
    battery_sub_ = node->create_subscription<sensor_msgs::msg::BatteryState>(
      battery_topic_,
      rclcpp::SystemDefaultsQoS(),
      std::bind(&IsBatteryChargingCondition::batteryCallback, this, std::placeholders::_1),
      sub_option);
  }
}

BT::NodeStatus IsBatteryChargingCondition::tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
  }

  callback_group_executor_.spin_some();
  return is_battery_charging_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void IsBatteryChargingCondition::batteryCallback(sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  is_battery_charging_ =
    (msg->power_supply_status == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING);
}

}  // namespace nav3d_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav3d_behavior_tree::IsBatteryChargingCondition>("IsBatteryCharging");
}