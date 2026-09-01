#pragma once

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav3d_util/lifecycle_node.hpp"
#include "nav3d_util/node_utils.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

namespace nav3d_util {

class OdomSmoother {
  public:
    explicit OdomSmoother(const rclcpp::Node::WeakPtr& parent, double filter_duration = 0.3,
                          const std::string& odom_topic = "odom");

    explicit OdomSmoother(const nav3d_util::LifecycleNode::WeakPtr& parent, double filter_duration = 0.3,
                          const std::string& odom_topic = "odom");

    geometry_msgs::msg::Twist getTwist() {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        if (!received_odom_) {
            RCLCPP_ERROR(rclcpp::get_logger("OdomSmoother"),
                         "OdomSmoother has not received any data yet, returning empty Twist");
            return geometry_msgs::msg::Twist{};
        }
        return vel_smooth_.twist;
    }

    geometry_msgs::msg::TwistStamped getTwistStamped() {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        if (!received_odom_) {
            RCLCPP_ERROR(rclcpp::get_logger("OdomSmoother"),
                         "OdomSmoother has not received any data yet, returning empty Twist");
            return geometry_msgs::msg::TwistStamped{};
        }
        return vel_smooth_;
    }

    geometry_msgs::msg::Twist getRawTwist() {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        if (!received_odom_) {
            RCLCPP_ERROR(rclcpp::get_logger("OdomSmoother"),
                         "OdomSmoother has not received any data yet, returning empty Twist");
            return geometry_msgs::msg::Twist{};
        }
        return odom_history_.back().twist.twist;
    }

    geometry_msgs::msg::TwistStamped getRawTwistStamped() {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        if (!received_odom_) {
            RCLCPP_ERROR(rclcpp::get_logger("OdomSmoother"),
                         "OdomSmoother has not received any data yet, returning empty Twist");
            return geometry_msgs::msg::TwistStamped{};
        }

        geometry_msgs::msg::TwistStamped twist_stamped;
        twist_stamped.header = odom_history_.back().header;
        twist_stamped.twist = odom_history_.back().twist.twist;
        return twist_stamped;
    }

  protected:
    void odomCallback(nav_msgs::msg::Odometry::SharedPtr msg);
    void updateState();

    bool received_odom_{false};
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    nav_msgs::msg::Odometry odom_cumulate_;
    geometry_msgs::msg::TwistStamped vel_smooth_;
    std::mutex odom_mutex_;

    rclcpp::Duration odom_history_duration_{0, 0};
    std::deque<nav_msgs::msg::Odometry> odom_history_;
};

} // namespace nav3d_util
