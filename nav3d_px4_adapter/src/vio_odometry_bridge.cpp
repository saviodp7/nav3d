#include "nav3d_px4_adapter/vio_odometry_bridge.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace nav3d_px4_adapter {

VioOdometryBridge::VioOdometryBridge(rclcpp::Node& node) : node_(node) {
    px4_topic_ = node_.declare_parameter<std::string>("px4_topic", "/fmu/in/vehicle_visual_odometry");

    odom_frame_ = node_.declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = node_.declare_parameter<std::string>("base_frame", "base_link");

    pose_frame_ = node_.declare_parameter<int>("pose_frame", 1);
    velocity_frame_ = node_.declare_parameter<int>("velocity_frame", 1);

    auto px4_qos = rclcpp::QoS(10).best_effort().durability_volatile();
    px4_pub_ = node_.create_publisher<px4_msgs::msg::VehicleOdometry>(px4_topic_, px4_qos);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

    R_enu2ned_ << 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0;

    R_flu2frd_ = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
}

void VioOdometryBridge::publishOdometry(const nav_msgs::msg::Odometry::SharedPtr msg) {
    const Eigen::Vector3d p_enu(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

    const Eigen::Quaterniond q_enu_flu(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                       msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

    const Eigen::Vector3d p_ned = R_enu2ned_ * p_enu;
    const Eigen::Matrix3d R_enu_flu = q_enu_flu.toRotationMatrix();
    const Eigen::Matrix3d R_ned_frd = R_enu2ned_ * R_enu_flu * R_flu2frd_.transpose();
    const Eigen::Quaterniond q_ned_frd(R_ned_frd);

    px4_msgs::msg::VehicleOdometry odom_px4;
    odom_px4.timestamp = now_us_steady();
    odom_px4.timestamp_sample = odom_px4.timestamp;

    odom_px4.pose_frame = static_cast<uint8_t>(pose_frame_);
    odom_px4.velocity_frame = static_cast<uint8_t>(velocity_frame_);

    odom_px4.position[0] = static_cast<float>(p_ned.x());
    odom_px4.position[1] = static_cast<float>(p_ned.y());
    odom_px4.position[2] = static_cast<float>(p_ned.z());

    odom_px4.q[0] = static_cast<float>(q_ned_frd.w());
    odom_px4.q[1] = static_cast<float>(q_ned_frd.x());
    odom_px4.q[2] = static_cast<float>(q_ned_frd.y());
    odom_px4.q[3] = static_cast<float>(q_ned_frd.z());

    odom_px4.velocity[0] = NAN;
    odom_px4.velocity[1] = NAN;
    odom_px4.velocity[2] = NAN;

    odom_px4.angular_velocity[0] = NAN;
    odom_px4.angular_velocity[1] = NAN;
    odom_px4.angular_velocity[2] = NAN;

    px4_pub_->publish(odom_px4);

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = msg->header.stamp;
    t.header.frame_id = odom_frame_;
    t.child_frame_id = base_frame_;
    t.transform.translation.x = msg->pose.pose.position.x;
    t.transform.translation.y = msg->pose.pose.position.y;
    t.transform.translation.z = msg->pose.pose.position.z;
    t.transform.rotation = msg->pose.pose.orientation;
    tf_broadcaster_->sendTransform(t);
}

uint64_t VioOdometryBridge::now_us_steady() {
    using clock = std::chrono::steady_clock;
    return std::chrono::time_point_cast<std::chrono::microseconds>(clock::now()).time_since_epoch().count();
}

} // namespace nav3d_px4_adapter
