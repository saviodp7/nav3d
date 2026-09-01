#pragma once

#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav3d_util/node_utils.hpp"
#include "nav3d_util/service_client.hpp"

#include <chrono>
#include <memory>
#include <string>

namespace nav3d_util {

class LifecycleServiceClient {
  public:
    explicit LifecycleServiceClient(const std::string& lifecycle_node_name);

    LifecycleServiceClient(const std::string& lifecycle_node_name, rclcpp::Node::SharedPtr parent_node);

    bool change_state(uint8_t transition, std::chrono::milliseconds transition_timeout = std::chrono::milliseconds(-1),
                      std::chrono::milliseconds wait_for_service_timeout = std::chrono::milliseconds(5000));

    uint8_t get_state(std::chrono::milliseconds timeout = std::chrono::milliseconds(2000));

  protected:
    rclcpp::Node::SharedPtr node_;
    ServiceClient<lifecycle_msgs::srv::ChangeState> change_state_;
    ServiceClient<lifecycle_msgs::srv::GetState> get_state_;
};

} // namespace nav3d_util
