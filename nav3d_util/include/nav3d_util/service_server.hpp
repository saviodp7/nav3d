#pragma once

#include "rclcpp/rclcpp.hpp"

#include <functional>
#include <memory>
#include <string>

namespace nav3d_util {

template <class ServiceT, typename NodeT = rclcpp::Node::SharedPtr> class ServiceServer {
  public:
    using RequestType = typename ServiceT::Request;
    using ResponseType = typename ServiceT::Response;
    using CallbackType = std::function<void(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<RequestType>,
                                            std::shared_ptr<ResponseType>)>;
    using SharedPtr = std::shared_ptr<ServiceServer<ServiceT, NodeT>>;

    explicit ServiceServer(const std::string& service_name, const NodeT& node, CallbackType callback,
                           const rclcpp::QoS& qos = rclcpp::ServicesQoS(),
                           rclcpp::CallbackGroup::SharedPtr callback_group = nullptr)
        : service_name_(service_name), callback_(std::move(callback)) {
        server_ = node->template create_service<ServiceT>(
            service_name_,
            [this](const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<RequestType> request,
                   std::shared_ptr<ResponseType> response) { callback_(request_header, request, std::move(response)); },
            qos.get_rmw_qos_profile(), callback_group);
    }

    const std::string& getServiceName() const { return service_name_; }

  protected:
    std::string service_name_;
    CallbackType callback_;
    typename rclcpp::Service<ServiceT>::SharedPtr server_;
};

} // namespace nav3d_util
