// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <chrono>
#include <memory>
#include <string>

#include "nav3d_msgs/srv/manage_lifecycle_nodes.hpp"
#include "nav3d_util/service_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace nav3d_lifecycle_manager
{

enum class SystemStatus {ACTIVE, INACTIVE, TIMEOUT};

class LifecycleManagerClient
{
public:
  explicit LifecycleManagerClient(
    const std::string & name,
    std::shared_ptr<rclcpp::Node> parent_node);

  bool startup(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));
  bool shutdown(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));
  bool pause(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));
  bool resume(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));
  bool reset(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  SystemStatus is_active(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

protected:
  using ManageLifecycleNodes = nav3d_msgs::srv::ManageLifecycleNodes;

  bool callService(
    uint8_t command,
    const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  rclcpp::Node::SharedPtr node_;

  std::shared_ptr<nav3d_util::ServiceClient<ManageLifecycleNodes>> manager_client_;
  std::shared_ptr<nav3d_util::ServiceClient<std_srvs::srv::Trigger>> is_active_client_;
  std::string manage_service_name_;
  std::string active_service_name_;
};

}  // namespace nav3d_lifecycle_manager