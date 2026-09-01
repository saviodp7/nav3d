#pragma once

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/time.hpp"
#include "tf2/transform_datatypes.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"

#include <memory>
#include <string>

namespace nav3d_util {

[[nodiscard]] bool getCurrentPose(geometry_msgs::msg::PoseStamped& global_pose, tf2_ros::Buffer& tf_buffer,
                                  const std::string& global_frame = "map", const std::string& robot_frame = "base_link",
                                  double transform_timeout = 0.1, const rclcpp::Time& stamp = rclcpp::Time());

[[nodiscard]] bool transformPoseInTargetFrame(const geometry_msgs::msg::PoseStamped& input_pose,
                                              geometry_msgs::msg::PoseStamped& transformed_pose,
                                              tf2_ros::Buffer& tf_buffer, const std::string& target_frame,
                                              double transform_timeout = 0.1);

[[nodiscard]] bool getTransform(const std::string& source_frame_id, const std::string& target_frame_id,
                                const tf2::Duration& transform_tolerance,
                                const std::shared_ptr<tf2_ros::Buffer> tf_buffer, tf2::Transform& tf2_transform);

[[nodiscard]] bool getTransform(const std::string& source_frame_id, const rclcpp::Time& source_time,
                                const std::string& target_frame_id, const rclcpp::Time& target_time,
                                const std::string& fixed_frame_id, const tf2::Duration& transform_tolerance,
                                const std::shared_ptr<tf2_ros::Buffer> tf_buffer, tf2::Transform& tf2_transform);

[[nodiscard]] bool getTransform(const std::string& source_frame_id, const std::string& target_frame_id,
                                const tf2::Duration& transform_tolerance,
                                const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                                geometry_msgs::msg::TransformStamped& transform_msg);

[[nodiscard]] bool getTransform(const std::string& source_frame_id, const rclcpp::Time& source_time,
                                const std::string& target_frame_id, const rclcpp::Time& target_time,
                                const std::string& fixed_frame_id, const tf2::Duration& transform_tolerance,
                                const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                                geometry_msgs::msg::TransformStamped& transform_msg);

[[nodiscard]] bool validateTwist(const geometry_msgs::msg::Twist& msg);

} // namespace nav3d_util
