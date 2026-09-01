#include "nav3d_util/odometry_utils.hpp"

#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

namespace nav3d_util {

OdomSmoother::OdomSmoother(const rclcpp::Node::WeakPtr& parent, double filter_duration, const std::string& odom_topic)
    : odom_history_duration_(rclcpp::Duration::from_seconds(filter_duration)) {
    auto node = parent.lock();
    if (!node) {
        throw std::runtime_error("OdomSmoother: parent node expired");
    }

    odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, rclcpp::SystemDefaultsQoS(), std::bind(&OdomSmoother::odomCallback, this, std::placeholders::_1));

    odom_cumulate_ = nav_msgs::msg::Odometry{};
}

OdomSmoother::OdomSmoother(const nav3d_util::LifecycleNode::WeakPtr& parent, double filter_duration,
                           const std::string& odom_topic)
    : odom_history_duration_(rclcpp::Duration::from_seconds(filter_duration)) {
    auto node = parent.lock();
    if (!node) {
        throw std::runtime_error("OdomSmoother: parent lifecycle node expired");
    }

    odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, rclcpp::SystemDefaultsQoS(), std::bind(&OdomSmoother::odomCallback, this, std::placeholders::_1));

    odom_cumulate_ = nav_msgs::msg::Odometry{};
}

void OdomSmoother::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    received_odom_ = true;

    if (!odom_history_.empty()) {
        auto current_time = rclcpp::Time(msg->header.stamp);
        auto front_time = rclcpp::Time(odom_history_.front().header.stamp);

        while ((current_time - front_time) > odom_history_duration_) {
            const auto& odom = odom_history_.front();
            odom_cumulate_.twist.twist.linear.x -= odom.twist.twist.linear.x;
            odom_cumulate_.twist.twist.linear.y -= odom.twist.twist.linear.y;
            odom_cumulate_.twist.twist.linear.z -= odom.twist.twist.linear.z;
            odom_cumulate_.twist.twist.angular.x -= odom.twist.twist.angular.x;
            odom_cumulate_.twist.twist.angular.y -= odom.twist.twist.angular.y;
            odom_cumulate_.twist.twist.angular.z -= odom.twist.twist.angular.z;
            odom_history_.pop_front();

            if (odom_history_.empty()) {
                break;
            }

            front_time = rclcpp::Time(odom_history_.front().header.stamp);
        }
    }

    odom_history_.push_back(*msg);
    updateState();
}

void OdomSmoother::updateState() {
    const auto& odom = odom_history_.back();

    odom_cumulate_.twist.twist.linear.x += odom.twist.twist.linear.x;
    odom_cumulate_.twist.twist.linear.y += odom.twist.twist.linear.y;
    odom_cumulate_.twist.twist.linear.z += odom.twist.twist.linear.z;
    odom_cumulate_.twist.twist.angular.x += odom.twist.twist.angular.x;
    odom_cumulate_.twist.twist.angular.y += odom.twist.twist.angular.y;
    odom_cumulate_.twist.twist.angular.z += odom.twist.twist.angular.z;

    vel_smooth_.header = odom.header;

    const auto n = static_cast<double>(odom_history_.size());
    vel_smooth_.twist.linear.x = odom_cumulate_.twist.twist.linear.x / n;
    vel_smooth_.twist.linear.y = odom_cumulate_.twist.twist.linear.y / n;
    vel_smooth_.twist.linear.z = odom_cumulate_.twist.twist.linear.z / n;
    vel_smooth_.twist.angular.x = odom_cumulate_.twist.twist.angular.x / n;
    vel_smooth_.twist.angular.y = odom_cumulate_.twist.twist.angular.y / n;
    vel_smooth_.twist.angular.z = odom_cumulate_.twist.twist.angular.z / n;
}

} // namespace nav3d_util
