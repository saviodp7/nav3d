#include "nav3d_util/robot_utils.hpp"

#include "rclcpp/logger.hpp"
#include "tf2/convert.hpp"

#include <cmath>
#include <memory>
#include <string>

namespace nav3d_util {

bool getCurrentPose(geometry_msgs::msg::PoseStamped& global_pose, tf2_ros::Buffer& tf_buffer,
                    const std::string& global_frame, const std::string& robot_frame, const double transform_timeout,
                    const rclcpp::Time& stamp) {
    tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
    global_pose.header.frame_id = robot_frame;
    global_pose.header.stamp = stamp;

    return transformPoseInTargetFrame(global_pose, global_pose, tf_buffer, global_frame, transform_timeout);
}

bool transformPoseInTargetFrame(const geometry_msgs::msg::PoseStamped& input_pose,
                                geometry_msgs::msg::PoseStamped& transformed_pose, tf2_ros::Buffer& tf_buffer,
                                const std::string& target_frame, const double transform_timeout) {
    const auto logger = rclcpp::get_logger("transformPoseInTargetFrame");

    try {
        transformed_pose = tf_buffer.transform(input_pose, target_frame, tf2::durationFromSec(transform_timeout));
        return true;
    } catch (tf2::LookupException& ex) {
        RCLCPP_ERROR(logger, "Lookup error transforming %s -> %s: %s", input_pose.header.frame_id.c_str(),
                     target_frame.c_str(), ex.what());
    } catch (tf2::ConnectivityException& ex) {
        RCLCPP_ERROR(logger, "Connectivity error transforming %s -> %s: %s", input_pose.header.frame_id.c_str(),
                     target_frame.c_str(), ex.what());
    } catch (tf2::ExtrapolationException& ex) {
        RCLCPP_ERROR(logger, "Extrapolation error transforming %s -> %s: %s", input_pose.header.frame_id.c_str(),
                     target_frame.c_str(), ex.what());
    } catch (tf2::TimeoutException& ex) {
        RCLCPP_ERROR(logger, "Timeout transforming %s -> %s (timeout=%.4f): %s", input_pose.header.frame_id.c_str(),
                     target_frame.c_str(), transform_timeout, ex.what());
    } catch (tf2::TransformException& ex) {
        RCLCPP_ERROR(logger, "Transform error transforming %s -> %s: %s", input_pose.header.frame_id.c_str(),
                     target_frame.c_str(), ex.what());
    }

    return false;
}

bool getTransform(const std::string& source_frame_id, const std::string& target_frame_id,
                  const tf2::Duration& transform_tolerance, const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                  geometry_msgs::msg::TransformStamped& transform_msg) {
    if (source_frame_id == target_frame_id) {
        transform_msg.header.frame_id = target_frame_id;
        transform_msg.child_frame_id = source_frame_id;
        transform_msg.transform = tf2::toMsg(tf2::Transform::getIdentity());
        return true;
    }

    try {
        transform_msg =
            tf_buffer->lookupTransform(target_frame_id, source_frame_id, tf2::TimePointZero, transform_tolerance);
        return true;
    } catch (tf2::TransformException& ex) {
        RCLCPP_ERROR(rclcpp::get_logger("getTransform"), "Failed to get \"%s\"->\"%s\" frame transform: %s",
                     source_frame_id.c_str(), target_frame_id.c_str(), ex.what());
        return false;
    }
}

bool getTransform(const std::string& source_frame_id, const std::string& target_frame_id,
                  const tf2::Duration& transform_tolerance, const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                  tf2::Transform& tf2_transform) {
    tf2_transform.setIdentity();
    geometry_msgs::msg::TransformStamped transform_msg;
    if (!getTransform(source_frame_id, target_frame_id, transform_tolerance, tf_buffer, transform_msg)) {
        return false;
    }
    tf2::fromMsg(transform_msg.transform, tf2_transform);
    return true;
}

bool getTransform(const std::string& source_frame_id, const rclcpp::Time& source_time,
                  const std::string& target_frame_id, const rclcpp::Time& target_time,
                  const std::string& fixed_frame_id, const tf2::Duration& transform_tolerance,
                  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                  geometry_msgs::msg::TransformStamped& transform_msg) {
    try {
        transform_msg = tf_buffer->lookupTransform(target_frame_id, target_time, source_frame_id, source_time,
                                                   fixed_frame_id, transform_tolerance);
        return true;
    } catch (tf2::TransformException& ex) {
        RCLCPP_ERROR(rclcpp::get_logger("getTransform"), "Failed to get \"%s\"->\"%s\" frame transform: %s",
                     source_frame_id.c_str(), target_frame_id.c_str(), ex.what());
        return false;
    }
}

bool getTransform(const std::string& source_frame_id, const rclcpp::Time& source_time,
                  const std::string& target_frame_id, const rclcpp::Time& target_time,
                  const std::string& fixed_frame_id, const tf2::Duration& transform_tolerance,
                  const std::shared_ptr<tf2_ros::Buffer> tf_buffer, tf2::Transform& tf2_transform) {
    tf2_transform.setIdentity();
    geometry_msgs::msg::TransformStamped transform_msg;
    if (!getTransform(source_frame_id, source_time, target_frame_id, target_time, fixed_frame_id, transform_tolerance,
                      tf_buffer, transform_msg)) {
        return false;
    }

    tf2::fromMsg(transform_msg.transform, tf2_transform);
    return true;
}

bool validateTwist(const geometry_msgs::msg::Twist& msg) {
    const auto finite = [](double v) { return !std::isnan(v) && !std::isinf(v); };

    return finite(msg.linear.x) && finite(msg.linear.y) && finite(msg.linear.z) && finite(msg.angular.x) &&
           finite(msg.angular.y) && finite(msg.angular.z);
}

} // namespace nav3d_util
