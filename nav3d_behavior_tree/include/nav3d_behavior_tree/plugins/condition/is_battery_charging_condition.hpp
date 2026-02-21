#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

namespace nav3d_behavior_tree
{

class IsBatteryChargingCondition : public BT::ConditionNode
{
public:
  IsBatteryChargingCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsBatteryChargingCondition() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "battery_topic", std::string("/battery_status"), "Battery topic")
    };
  }

private:
  void initialize();
  void createROSInterfaces();

  void batteryCallback(sensor_msgs::msg::BatteryState::SharedPtr msg);

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  std::string battery_topic_;
  bool is_battery_charging_;
};

}  // namespace nav3d_behavior_tree