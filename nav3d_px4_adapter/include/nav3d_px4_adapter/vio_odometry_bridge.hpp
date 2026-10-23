#pragma once

#include <Eigen/Dense>
#include <cstdint>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_ros/transform_broadcaster.h>

class VioOdometryBridge final : public rclcpp::Node {
  public:
    VioOdometryBridge();

  private:
    void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
    static uint64_t now_us_steady();

    std::string odom_topic_;
    std::string px4_topic_;

    std::string odom_frame_;
    std::string base_frame_;

    int pose_frame_{1};
    int velocity_frame_{1};

    Eigen::Matrix3d R_enu2ned_;
    Eigen::Matrix3d R_flu2frd_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};
