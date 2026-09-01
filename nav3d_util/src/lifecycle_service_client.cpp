#include "nav3d_util/lifecycle_service_client.hpp"

#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

#include <chrono>
#include <memory>
#include <string>

using nav3d_util::generate_internal_node;
using std::string;
using std::chrono::milliseconds;
using namespace std::chrono_literals;

namespace nav3d_util {

LifecycleServiceClient::LifecycleServiceClient(const string& lifecycle_node_name)
    : node_(generate_internal_node(lifecycle_node_name + "_lifecycle_client")),
      change_state_(lifecycle_node_name + "/change_state", node_, true),
      get_state_(lifecycle_node_name + "/get_state", node_, true) {
    rclcpp::Rate r(20);
    while (!get_state_.wait_for_service(2s)) {
        RCLCPP_INFO(node_->get_logger(), "Waiting for service %s...", get_state_.getServiceName().c_str());
        r.sleep();
    }
}

LifecycleServiceClient::LifecycleServiceClient(const string& lifecycle_node_name, rclcpp::Node::SharedPtr parent_node)
    : node_(std::move(parent_node)), change_state_(lifecycle_node_name + "/change_state", node_, true),
      get_state_(lifecycle_node_name + "/get_state", node_, true) {
    rclcpp::Rate r(20);
    while (!get_state_.wait_for_service(2s)) {
        RCLCPP_INFO(node_->get_logger(), "Waiting for service %s...", get_state_.getServiceName().c_str());
        r.sleep();
    }
}

bool LifecycleServiceClient::change_state(const uint8_t transition, const milliseconds transition_timeout,
                                          const milliseconds wait_for_service_timeout) {
    if (!change_state_.wait_for_service(wait_for_service_timeout)) {
        throw std::runtime_error("change_state service is not available!");
    }

    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;

    if (transition_timeout > 0ms) {
        auto response = change_state_.invoke(request, transition_timeout);
        return response.get();
    }

    auto response = std::make_shared<lifecycle_msgs::srv::ChangeState::Response>();
    return change_state_.invoke(request, response);
}

uint8_t LifecycleServiceClient::get_state(const milliseconds timeout) {
    if (!get_state_.wait_for_service(timeout)) {
        throw std::runtime_error("get_state service is not available!");
    }

    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    auto result = get_state_.invoke(request, timeout);
    return result->current_state.id;
}

} // namespace nav3d_util
