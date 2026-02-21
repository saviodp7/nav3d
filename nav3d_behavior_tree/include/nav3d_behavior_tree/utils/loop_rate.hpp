#pragma once

#include <memory>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

namespace nav3d_behavior_tree
{

class LoopRate
{
public:
  LoopRate(const rclcpp::Duration & period, BT::Tree * tree)
  : clock_(std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME)),
    period_(period),
    last_interval_(clock_->now()),
    tree_(tree)
  {}

  bool sleep()
  {
    auto now = clock_->now();
    auto next_interval = last_interval_ + period_;

    if (now < last_interval_) {
      next_interval = now + period_;
    }

    last_interval_ += period_;

    if (next_interval <= now) {
      if (now > next_interval + period_) {
        last_interval_ = now + period_;
      }
      return false;
    }

    auto time_to_sleep = next_interval - now;
    std::chrono::nanoseconds time_to_sleep_ns(time_to_sleep.nanoseconds());
    tree_->sleep(time_to_sleep_ns);
    return true;
  }

  std::chrono::nanoseconds period() const
  {
    return std::chrono::nanoseconds(period_.nanoseconds());
  }

private:
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Duration period_;
  rclcpp::Time last_interval_;
  BT::Tree * tree_;
};

}  // namespace nav3d_behavior_tree