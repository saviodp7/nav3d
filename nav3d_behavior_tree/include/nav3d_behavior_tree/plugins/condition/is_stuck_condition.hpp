#pragma once

#include <atomic>
#include <deque>
#include <string>
#include <thread>

#include "behaviortree_cpp/condition_node.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav3d_behavior_tree
{

class IsStuckCondition : public BT::ConditionNode
{
public:
  IsStuckCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsStuckCondition() = delete;

  ~IsStuckCondition() override;

  void onOdomReceived(const nav_msgs::msg::Odometry::SharedPtr msg);

  BT::NodeStatus tick() override;

  void logStuck(const std::string & msg) const;

  void updateStates();

  bool isStuck();

  static BT::PortsList providedPorts()
  {
    return {};
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  std::thread callback_group_executor_thread;

  std::atomic<bool> is_stuck_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::deque<nav_msgs::msg::Odometry> odom_history_;
  std::deque<nav_msgs::msg::Odometry>::size_type odom_history_size_;

  double current_accel_;

  double brake_accel_limit_;
};

}  // namespace nav3d_behavior_tree