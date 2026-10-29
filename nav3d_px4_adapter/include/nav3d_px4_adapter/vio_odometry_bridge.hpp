#pragma once

#include <Eigen/Dense>
#include <cstdint>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_ros/transform_broadcaster.h>

namespace nav3d_px4_adapter {

// PX4 odometry conversion and the temporary odom -> base_link TF owner.
class VioOdometryBridge final {
  public:
    explicit VioOdometryBridge(rclcpp::Node& node);
    void publishOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);

  private:
    static uint64_t now_us_steady();

    rclcpp::Node& node_;
    std::string px4_topic_;

    std::string odom_frame_;
    std::string base_frame_;

    int pose_frame_{1};
    int velocity_frame_{1};

    Eigen::Matrix3d R_enu2ned_;
    Eigen::Matrix3d R_flu2frd_;

    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

} // namespace nav3d_px4_adapter
