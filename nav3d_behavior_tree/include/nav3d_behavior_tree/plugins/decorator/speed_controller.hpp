#pragma once

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/decorator_node.h"
#include "behaviortree_cpp/json_export.h"
#include "nav3d_behavior_tree/bt_utils.hpp"
#include "nav3d_behavior_tree/json_utils.hpp"
#include "nav3d_util/odometry_utils.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace nav3d_behavior_tree
{

class SpeedController : public BT::DecoratorNode
{
public:
  SpeedController(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    BT::RegisterJsonDefinition<geometry_msgs::msg::PoseStamped>();
    BT::RegisterJsonDefinition<nav_msgs::msg::Goals>();

    return {
      BT::InputPort<double>("min_rate", 0.1, "Minimum rate"),
      BT::InputPort<double>("max_rate", 1.0, "Maximum rate"),
      BT::InputPort<double>("min_speed", 0.0, "Minimum speed"),
      BT::InputPort<double>("max_speed", 0.5, "Maximum speed"),
      BT::InputPort<nav_msgs::msg::Goals>(
        "goals", "Vector of navigation goals"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "goal", "Navigation goal"),
    };
  }

private:
  BT::NodeStatus tick() override;

  inline double getScaledRate(const double & speed)
  {
    return std::max(
      std::min(
        (((speed - min_speed_) / d_speed_) * d_rate_) + min_rate_,
        max_rate_),
      min_rate_);
  }

  inline void updatePeriod()
  {
    auto velocity = odom_smoother_->getTwist();
    double speed = std::hypot(velocity.linear.x, velocity.linear.y);
    double rate = getScaledRate(speed);
    period_ = 1.0 / rate;
  }

  rclcpp::Node::SharedPtr node_;

  rclcpp::Time start_;

  std::shared_ptr<nav3d_util::OdomSmoother> odom_smoother_;

  bool first_tick_;

  double period_;

  double min_rate_;
  double max_rate_;
  double d_rate_;

  double min_speed_;
  double max_speed_;
  double d_speed_;

  geometry_msgs::msg::PoseStamped goal_;
  nav_msgs::msg::Goals goals_;
};

}  // namespace nav3d_behavior_tree