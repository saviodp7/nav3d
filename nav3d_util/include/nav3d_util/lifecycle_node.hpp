#pragma once

#include "bond/msg/constants.hpp"
#include "bondcpp/bond.hpp"
#include "nav3d_util/node_thread.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <memory>
#include <string>
#include <thread>

namespace nav3d_util {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class LifecycleNode : public rclcpp_lifecycle::LifecycleNode {
  public:
    LifecycleNode(const std::string& node_name, const std::string& ns = "",
                  const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    ~LifecycleNode() override;

    struct FloatingPointRange {
        double from_value;
        double to_value;
        double step;
    };

    struct IntegerRange {
        int from_value;
        int to_value;
        int step;
    };

    void add_parameter(const std::string& name, const rclcpp::ParameterValue& default_value,
                       const std::string& description = "", const std::string& additional_constraints = "",
                       bool read_only = false) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.name = name;
        descriptor.description = description;
        descriptor.additional_constraints = additional_constraints;
        descriptor.read_only = read_only;

        declare_parameter(descriptor.name, default_value, descriptor);
    }

    void add_parameter(const std::string& name, const rclcpp::ParameterValue& default_value,
                       const FloatingPointRange fp_range, const std::string& description = "",
                       const std::string& additional_constraints = "", bool read_only = false) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.name = name;
        descriptor.description = description;
        descriptor.additional_constraints = additional_constraints;
        descriptor.read_only = read_only;

        descriptor.floating_point_range.resize(1);
        descriptor.floating_point_range[0].from_value = fp_range.from_value;
        descriptor.floating_point_range[0].to_value = fp_range.to_value;
        descriptor.floating_point_range[0].step = fp_range.step;

        declare_parameter(descriptor.name, default_value, descriptor);
    }

    void add_parameter(const std::string& name, const rclcpp::ParameterValue& default_value,
                       const IntegerRange int_range, const std::string& description = "",
                       const std::string& additional_constraints = "", bool read_only = false) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.name = name;
        descriptor.description = description;
        descriptor.additional_constraints = additional_constraints;
        descriptor.read_only = read_only;

        descriptor.integer_range.resize(1);
        descriptor.integer_range[0].from_value = int_range.from_value;
        descriptor.integer_range[0].to_value = int_range.to_value;
        descriptor.integer_range[0].step = int_range.step;

        declare_parameter(descriptor.name, default_value, descriptor);
    }

    [[nodiscard]] std::shared_ptr<nav3d_util::LifecycleNode> shared_from_this() {
        return std::static_pointer_cast<nav3d_util::LifecycleNode>(rclcpp_lifecycle::LifecycleNode::shared_from_this());
    }

    nav3d_util::CallbackReturn on_error(const rclcpp_lifecycle::State& /*state*/) {
        RCLCPP_FATAL(get_logger(), "Lifecycle node %s does not have error state implemented", get_name());
        return nav3d_util::CallbackReturn::SUCCESS;
    }

    void autostart();

    virtual void on_rcl_preshutdown();

    void createBond();
    void destroyBond();

  protected:
    void printLifecycleNodeNotification();

    void register_rcl_preshutdown_callback();
    std::unique_ptr<rclcpp::PreShutdownCallbackHandle> rcl_preshutdown_cb_handle_{nullptr};

    void runCleanups();

    std::unique_ptr<bond::Bond> bond_{nullptr};
    double bond_heartbeat_period;
    rclcpp::TimerBase::SharedPtr autostart_timer_;
};

} // namespace nav3d_util
