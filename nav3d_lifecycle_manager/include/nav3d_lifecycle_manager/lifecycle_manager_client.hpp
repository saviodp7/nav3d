#pragma once

#include "nav3d_msgs/srv/manage_lifecycle_nodes.hpp"
#include "nav3d_util/service_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <chrono>
#include <memory>
#include <string>

namespace nav3d_lifecycle_manager {

enum class SystemStatus { ACTIVE, INACTIVE, TIMEOUT };

class LifecycleManagerClient {
  public:
    explicit LifecycleManagerClient(const std::string& name, std::shared_ptr<rclcpp::Node> parent_node);

    bool startup(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));
    bool shutdown(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));
    bool pause(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));
    bool resume(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));
    bool reset(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

    SystemStatus is_active(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  protected:
    using ManageLifecycleNodes = nav3d_msgs::srv::ManageLifecycleNodes;

    bool callService(uint8_t command, const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

    rclcpp::Node::SharedPtr node_;

    std::shared_ptr<nav3d_util::ServiceClient<ManageLifecycleNodes>> manager_client_;
    std::shared_ptr<nav3d_util::ServiceClient<std_srvs::srv::Trigger>> is_active_client_;
    std::string manage_service_name_;
    std::string active_service_name_;
};

} // namespace nav3d_lifecycle_manager