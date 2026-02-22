#include "nav3d_behavior_tree/plugins/condition/is_path_valid_condition.hpp"

#include <chrono>
#include <memory>
#include <string>

namespace nav3d_behavior_tree
{

IsPathValidCondition::IsPathValidCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  client_ = node_->create_client<nav3d_msgs::srv::IsPathValid>(service_name_);

  server_timeout_ = config().blackboard->get<std::chrono::milliseconds>("server_timeout");
  getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);
}

BT::NodeStatus IsPathValidCondition::tick()
{
  nav_msgs::msg::Path path;
  if (!getInput("path", path)) {
    return BT::NodeStatus::FAILURE;
  }

  std::string service_name = service_name_;
  getInput("service_name", service_name);
  if (service_name != service_name_) {
    service_name_ = std::move(service_name);
    client_ = node_->create_client<nav3d_msgs::srv::IsPathValid>(service_name_);
  }

  auto request = std::make_shared<nav3d_msgs::srv::IsPathValid::Request>();
  request->path = path;

  auto future = client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future, server_timeout_) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    if (future.get()->is_valid) {
      return BT::NodeStatus::SUCCESS;
    }
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace nav3d_behavior_tree

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav3d_behavior_tree::IsPathValidCondition>("IsPathValid");
}