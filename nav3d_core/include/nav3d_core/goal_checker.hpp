#pragma once

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <memory>
#include <string>

namespace nav3d_core {

class GoalChecker {
  public:
    using Ptr = std::shared_ptr<nav3d_core::GoalChecker>;

    virtual ~GoalChecker() = default;

    virtual void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, const std::string& plugin_name) = 0;

    virtual void reset() = 0;

    virtual bool isGoalReached(const geometry_msgs::msg::Pose& query_pose, const geometry_msgs::msg::Pose& goal_pose,
                               const geometry_msgs::msg::Twist& velocity) = 0;

    virtual bool getTolerances(geometry_msgs::msg::Pose& pose_tolerance, geometry_msgs::msg::Twist& vel_tolerance) = 0;
};

} // namespace nav3d_core
