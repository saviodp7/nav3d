#pragma once

#include <chrono>
#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/json_export.h"
#include "nav3d_behavior_tree/bt_utils.hpp"
#include "nav3d_behavior_tree/json_utils.hpp"
#include "nav3d_util/node_utils.hpp"
#include "nav3d_util/service_client.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav3d_behavior_tree
{

using namespace std::chrono_literals;  // NOLINT

template<class ServiceT>
class BtServiceNode : public BT::ActionNodeBase
{
public:
  BtServiceNode(
    const std::string & service_node_name,
    const BT::NodeConfiguration & conf,
    const std::string & service_name = "")
  : BT::ActionNodeBase(service_node_name, conf),
    service_name_(service_name),
    service_node_name_(service_node_name)
  {
    initialize();

    request_ = std::make_shared<typename ServiceT::Request>();

    RCLCPP_DEBUG(
      node_->get_logger(), "Waiting for \"%s\" service",
      service_name_.c_str());

    if (!service_client_->wait_for_service(wait_for_service_timeout_)) {
      RCLCPP_ERROR(
        node_->get_logger(), "\"%s\" service server not available after waiting for %.2fs",
        service_name_.c_str(), wait_for_service_timeout_.count() / 1000.0);
      throw std::runtime_error(
        std::string("Service server ") + service_name_ + std::string(" not available"));
    }

    RCLCPP_DEBUG(
      node_->get_logger(), "\"%s\" BtServiceNode initialized",
      service_node_name_.c_str());
  }

  BtServiceNode() = delete;

  virtual ~BtServiceNode() = default;

  void initialize()
  {
    auto bt_loop_duration =
      config().blackboard->template get<std::chrono::milliseconds>("bt_loop_duration");
    getInputOrBlackboard("server_timeout", server_timeout_);
    wait_for_service_timeout_ =
      config().blackboard->template get<std::chrono::milliseconds>("wait_for_service_timeout");

    max_timeout_ = std::chrono::duration_cast<std::chrono::milliseconds>(bt_loop_duration * 0.5);

    createROSInterfaces();
  }

  void createROSInterfaces()
  {
    std::string service_new;
    getInput("service_name", service_new);
    if (service_new != service_name_ || !service_client_) {
      service_name_ = service_new;
      node_ = config().blackboard->template get<rclcpp::Node::SharedPtr>("node");
      service_client_ = std::make_shared<nav3d_util::ServiceClient<ServiceT>>(
        service_name_, node_, true);
    }
  }

  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {
    BT::PortsList basic = {
      BT::InputPort<std::string>("service_name", "please_set_service_name_in_BT_Node"),
      BT::InputPort<std::chrono::milliseconds>("server_timeout")
    };
    basic.insert(addition.begin(), addition.end());
    return basic;
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  BT::NodeStatus tick() override
  {
    if (!BT::isStatusActive(status())) {
      initialize();
    }

    if (!request_sent_) {
      should_send_request_ = true;

      request_ = std::make_shared<typename ServiceT::Request>();

      on_tick();

      if (!should_send_request_) {
        return BT::NodeStatus::FAILURE;
      }

      future_result_ = service_client_->async_call(request_);
      sent_time_ = node_->now();
      request_sent_ = true;
    }

    return check_future();
  }

  void halt() override
  {
    request_sent_ = false;
    resetStatus();
  }

  virtual void on_tick() {}

  virtual BT::NodeStatus on_completion(std::shared_ptr<typename ServiceT::Response> /*response*/)
  {
    return BT::NodeStatus::SUCCESS;
  }

  virtual BT::NodeStatus check_future()
  {
    auto elapsed = (node_->now() - sent_time_).template to_chrono<std::chrono::milliseconds>();
    auto remaining = server_timeout_ - elapsed;

    if (remaining > std::chrono::milliseconds(0)) {
      auto timeout = remaining > max_timeout_ ? max_timeout_ : remaining;

      auto rc = service_client_->spin_until_complete(future_result_, timeout);
      if (rc == rclcpp::FutureReturnCode::SUCCESS) {
        request_sent_ = false;
        return on_completion(future_result_.get());
      }

      if (rc == rclcpp::FutureReturnCode::TIMEOUT) {
        on_wait_for_result();
        elapsed = (node_->now() - sent_time_).template to_chrono<std::chrono::milliseconds>();
        if (elapsed < server_timeout_) {
          return BT::NodeStatus::RUNNING;
        }
      }
    }

    RCLCPP_WARN(
      node_->get_logger(),
      "Node timed out while executing service call to %s.", service_name_.c_str());
    request_sent_ = false;
    return BT::NodeStatus::FAILURE;
  }

  virtual void on_wait_for_result() {}

protected:
  void increment_recovery_count()
  {
    int recovery_count = 0;
    [[maybe_unused]] auto res = config().blackboard->get("number_recoveries", recovery_count);
    recovery_count += 1;
    config().blackboard->set("number_recoveries", recovery_count);
  }

  std::string service_name_;
  std::string service_node_name_;

  typename nav3d_util::ServiceClient<ServiceT>::SharedPtr service_client_;
  std::shared_ptr<typename ServiceT::Request> request_;

  rclcpp::Node::SharedPtr node_;

  std::chrono::milliseconds server_timeout_;
  std::chrono::milliseconds max_timeout_;
  std::chrono::milliseconds wait_for_service_timeout_;

  std::shared_future<typename ServiceT::Response::SharedPtr> future_result_;
  bool request_sent_{false};
  rclcpp::Time sent_time_;

  bool should_send_request_;
};

}  // namespace nav3d_behavior_tree