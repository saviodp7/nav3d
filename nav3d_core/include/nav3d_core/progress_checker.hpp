#pragma once

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <memory>
#include <string>

namespace nav3d_core {

class ProgressChecker {
  public:
    using Ptr = std::shared_ptr<nav3d_core::ProgressChecker>;

    virtual ~ProgressChecker() = default;

    virtual void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, const std::string& plugin_name) = 0;

    virtual bool check(geometry_msgs::msg::PoseStamped& current_pose) = 0;

    virtual void reset() = 0;
};

} // namespace nav3d_core
