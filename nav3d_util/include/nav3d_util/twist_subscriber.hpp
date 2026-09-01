#pragma once

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav3d_util/lifecycle_node.hpp"
#include "nav3d_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <string>
#include <utility>

namespace nav3d_util {

class TwistSubscriber {
  public:
    template <typename TwistCallbackT, typename TwistStampedCallbackT>
    explicit TwistSubscriber(nav3d_util::LifecycleNode::SharedPtr node, const std::string& topic,
                             const rclcpp::QoS& qos, TwistCallbackT&& TwistCallback,
                             TwistStampedCallbackT&& TwistStampedCallback) {
        nav3d_util::declare_parameter_if_not_declared(node, "enable_stamped_cmd_vel", rclcpp::ParameterValue(true));
        node->get_parameter("enable_stamped_cmd_vel", is_stamped_);

        if (is_stamped_) {
            twist_stamped_sub_ = node->create_subscription<geometry_msgs::msg::TwistStamped>(
                topic, qos, std::forward<TwistStampedCallbackT>(TwistStampedCallback));
        } else {
            twist_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
                topic, qos, std::forward<TwistCallbackT>(TwistCallback));
        }
    }

    template <typename TwistStampedCallbackT>
    explicit TwistSubscriber(nav3d_util::LifecycleNode::SharedPtr node, const std::string& topic,
                             const rclcpp::QoS& qos, TwistStampedCallbackT&& TwistStampedCallback) {
        nav3d_util::declare_parameter_if_not_declared(node, "enable_stamped_cmd_vel", rclcpp::ParameterValue(true));
        node->get_parameter("enable_stamped_cmd_vel", is_stamped_);

        if (is_stamped_) {
            twist_stamped_sub_ = node->create_subscription<geometry_msgs::msg::TwistStamped>(
                topic, qos, std::forward<TwistStampedCallbackT>(TwistStampedCallback));
        } else {
            throw std::invalid_argument("enable_stamped_cmd_vel must be true when using this constructor!");
        }
    }

  protected:
    bool is_stamped_{true};
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_{nullptr};
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_stamped_sub_{nullptr};
};

} // namespace nav3d_util
