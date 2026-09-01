#pragma once

#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <utility>

namespace nav3d_util {

template <class ServiceT, typename NodeT = rclcpp::Node::SharedPtr> class ServiceClient {
  public:
    using RequestType = typename ServiceT::Request;
    using ResponseType = typename ServiceT::Response;
    using SharedPtr = std::shared_ptr<ServiceClient<ServiceT, NodeT>>;

    explicit ServiceClient(const std::string& service_name, const NodeT& provided_node,
                           bool use_internal_executor = false)
        : service_name_(service_name), node_(provided_node), use_internal_executor_(use_internal_executor) {
        if (use_internal_executor_) {
            callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
            callback_group_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
            callback_group_executor_->add_callback_group(callback_group_, node_->get_node_base_interface());
        }

        client_ = node_->template create_client<ServiceT>(
            service_name_, rclcpp::SystemDefaultsQoS().get_rmw_qos_profile(), callback_group_);
    }

    ~ServiceClient() {
        if (use_internal_executor_ && callback_group_executor_ && callback_group_) {
            callback_group_executor_->remove_callback_group(callback_group_);
        }
    }

    typename ResponseType::SharedPtr invoke(typename RequestType::SharedPtr& request,
                                            const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1)) {
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                throw std::runtime_error(service_name_ + " service client: interrupted while waiting for service");
            }
            RCLCPP_INFO(node_->get_logger(), "%s service client: waiting for service to appear...",
                        service_name_.c_str());
        }

        RCLCPP_DEBUG(node_->get_logger(), "%s service client: send async request", service_name_.c_str());

        auto future_result = client_->async_send_request(request);

        if (spin_until_complete(future_result, timeout) != rclcpp::FutureReturnCode::SUCCESS) {
            client_->remove_pending_request(future_result);
            throw std::runtime_error(service_name_ + " service client: async_send_request failed");
        }

        return future_result.get();
    }

    bool invoke(typename RequestType::SharedPtr& request, typename ResponseType::SharedPtr& response) {
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                throw std::runtime_error(service_name_ + " service client: interrupted while waiting for service");
            }
            RCLCPP_INFO(node_->get_logger(), "%s service client: waiting for service to appear...",
                        service_name_.c_str());
        }

        RCLCPP_DEBUG(node_->get_logger(), "%s service client: send async request", service_name_.c_str());

        auto future_result = client_->async_send_request(request);

        if (spin_until_complete(future_result) != rclcpp::FutureReturnCode::SUCCESS) {
            client_->remove_pending_request(future_result);
            return false;
        }

        response = future_result.get();
        return static_cast<bool>(response);
    }

    std::shared_future<typename ResponseType::SharedPtr> async_call(typename RequestType::SharedPtr& request) {
        return client_->async_send_request(request).share();
    }

    template <typename CallbackT> void async_call(typename RequestType::SharedPtr request, CallbackT&& callback) {
        client_->async_send_request(std::move(request), std::forward<CallbackT>(callback));
    }

    bool wait_for_service(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds::max()) {
        return client_->wait_for_service(timeout);
    }

    template <typename FutureT>
    rclcpp::FutureReturnCode
    spin_until_complete(const FutureT& future, const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1)) {
        if (use_internal_executor_) {
            return callback_group_executor_->spin_until_future_complete(future, timeout);
        }
        return rclcpp::spin_until_future_complete(node_, future, timeout);
    }

    const std::string& getServiceName() const { return service_name_; }

  protected:
    std::string service_name_;
    NodeT node_;
    rclcpp::CallbackGroup::SharedPtr callback_group_{nullptr};
    rclcpp::executors::SingleThreadedExecutor::SharedPtr callback_group_executor_;
    typename rclcpp::Client<ServiceT>::SharedPtr client_;
    bool use_internal_executor_{false};
};

} // namespace nav3d_util
