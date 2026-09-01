#pragma once

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav3d_core/goal_checker.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <memory>
#include <string>

namespace nav3d_core {

class Controller {
  public:
    using Ptr = std::shared_ptr<nav3d_core::Controller>;

    virtual ~Controller() = default;

    virtual void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, const std::string& name) = 0;

    virtual void cleanup() = 0;
    virtual void activate() = 0;
    virtual void deactivate() = 0;

    virtual void setPlan(const nav_msgs::msg::Path& path) = 0;

    virtual geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose,
                                                                     const geometry_msgs::msg::Twist& velocity,
                                                                     nav3d_core::GoalChecker* goal_checker) = 0;

    virtual bool cancel() { return true; }

    virtual void setSpeedLimit(const double& speed_limit, const bool& percentage) = 0;

    virtual void reset() {}
};

} // namespace nav3d_core
