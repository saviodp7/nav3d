#pragma once

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <string>

namespace nav3d_core {

class WaypointTaskExecutor {
  public:
    WaypointTaskExecutor() = default;
    virtual ~WaypointTaskExecutor() = default;

    virtual void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, const std::string& plugin_name) = 0;

    virtual bool processAtWaypoint(const geometry_msgs::msg::PoseStamped& curr_pose,
                                   const int& curr_waypoint_index) = 0;
};

} // namespace nav3d_core
