#pragma once

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav3d_util/lifecycle_node.hpp"
#include "nav3d_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include <memory>
#include <string>
#include <utility>

namespace nav3d_util {

class TwistPublisher {
  public:
    explicit TwistPublisher(nav3d_util::LifecycleNode::SharedPtr node, const std::string& topic, const rclcpp::QoS& qos)
        : topic_(topic) {
        using nav3d_util::declare_parameter_if_not_declared;
        declare_parameter_if_not_declared(node, "enable_stamped_cmd_vel", rclcpp::ParameterValue{true});
        node->get_parameter("enable_stamped_cmd_vel", is_stamped_);

        if (is_stamped_) {
            twist_stamped_pub_ = node->create_publisher<geometry_msgs::msg::TwistStamped>(topic_, qos);
        } else {
            twist_pub_ = node->create_publisher<geometry_msgs::msg::Twist>(topic_, qos);
        }
    }

    void on_activate() {
        if (is_stamped_) {
            twist_stamped_pub_->on_activate();
        } else {
            twist_pub_->on_activate();
        }
    }

    void on_deactivate() {
        if (is_stamped_) {
            twist_stamped_pub_->on_deactivate();
        } else {
            twist_pub_->on_deactivate();
        }
    }

    [[nodiscard]] bool is_activated() const {
        if (is_stamped_) {
            return twist_stamped_pub_->is_activated();
        }
        return twist_pub_->is_activated();
    }

    void publish(std::unique_ptr<geometry_msgs::msg::TwistStamped> velocity) {
        if (is_stamped_) {
            twist_stamped_pub_->publish(std::move(velocity));
            return;
        }

        auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>(velocity->twist);
        twist_pub_->publish(std::move(twist_msg));
    }

    [[nodiscard]] size_t get_subscription_count() const {
        if (is_stamped_) {
            return twist_stamped_pub_->get_subscription_count();
        }
        return twist_pub_->get_subscription_count();
    }

  protected:
    std::string topic_;
    bool is_stamped_{true};
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_stamped_pub_;
};

} // namespace nav3d_util
