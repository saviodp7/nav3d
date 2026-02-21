#pragma once

#include <chrono>
#include <string>
#include <vector>

#include "behaviortree_cpp/json_export.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav3d_msgs/msg/waypoint_status.hpp"
#include "nav_msgs/msg/goals.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/time.hpp"

namespace builtin_interfaces::msg
{

BT_JSON_CONVERTER(builtin_interfaces::msg::Time, msg)
{
  add_field("sec", &msg.sec);
  add_field("nanosec", &msg.nanosec);
}

}  // namespace builtin_interfaces::msg

namespace std_msgs::msg
{

BT_JSON_CONVERTER(std_msgs::msg::Header, msg)
{
  add_field("stamp", &msg.stamp);
  add_field("frame_id", &msg.frame_id);
}

}  // namespace std_msgs::msg

namespace geometry_msgs::msg
{

BT_JSON_CONVERTER(geometry_msgs::msg::Point, msg)
{
  add_field("x", &msg.x);
  add_field("y", &msg.y);
  add_field("z", &msg.z);
}

BT_JSON_CONVERTER(geometry_msgs::msg::Quaternion, msg)
{
  add_field("x", &msg.x);
  add_field("y", &msg.y);
  add_field("z", &msg.z);
  add_field("w", &msg.w);
}

BT_JSON_CONVERTER(geometry_msgs::msg::Pose, msg)
{
  add_field("position", &msg.position);
  add_field("orientation", &msg.orientation);
}

BT_JSON_CONVERTER(geometry_msgs::msg::PoseStamped, msg)
{
  add_field("header", &msg.header);
  add_field("pose", &msg.pose);
}

}  // namespace geometry_msgs::msg

namespace nav_msgs::msg
{

BT_JSON_CONVERTER(nav_msgs::msg::Goals, msg)
{
  add_field("header", &msg.header);
  add_field("goals", &msg.goals);
}

BT_JSON_CONVERTER(nav_msgs::msg::Path, msg)
{
  add_field("header", &msg.header);
  add_field("poses", &msg.poses);
}

}  // namespace nav_msgs::msg

namespace nav3d_msgs::msg
{

BT_JSON_CONVERTER(nav3d_msgs::msg::WaypointStatus, msg)
{
  add_field("waypoint_status", &msg.waypoint_status);
  add_field("waypoint_index", &msg.waypoint_index);
  add_field("waypoint_pose", &msg.waypoint_pose);
  add_field("error_code", &msg.error_code);
  add_field("error_msg", &msg.error_msg);
}

}  // namespace nav3d_msgs::msg

namespace std
{

inline void from_json(const nlohmann::json & js, std::chrono::milliseconds & dest)
{
  if (js.contains("ms")) {
    dest = std::chrono::milliseconds(js.at("ms").get<int>());
  } else {
    throw std::runtime_error("Invalid JSON for std::chrono::milliseconds");
  }
}

inline void to_json(nlohmann::json & js, const std::chrono::milliseconds & src)
{
  js["__type"] = "std::chrono::milliseconds";
  js["ms"] = src.count();
}

}  // namespace std