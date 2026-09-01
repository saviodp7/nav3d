#pragma once

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

#include <functional>
#include <memory>
#include <string>

namespace nav3d_core {

class GlobalPlanner {
  public:
    using Ptr = std::shared_ptr<GlobalPlanner>;

    virtual ~GlobalPlanner() = default;

    virtual void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name,
                           std::shared_ptr<tf2_ros::Buffer> tf) = 0;

    virtual void cleanup() = 0;
    virtual void activate() = 0;
    virtual void deactivate() = 0;

    virtual nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped& start,
                                           const geometry_msgs::msg::PoseStamped& goal,
                                           std::function<bool()> cancel_checker) = 0;
};

} // namespace nav3d_core
