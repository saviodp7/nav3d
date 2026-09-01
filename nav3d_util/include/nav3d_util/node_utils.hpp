#pragma once

#include "pluginlib/exceptions.hpp"
#include "rcl_interfaces/srv/list_parameters.hpp"
#include "rclcpp/rclcpp.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace nav3d_util {

[[nodiscard]] std::string sanitize_node_name(const std::string& potential_node_name);

[[nodiscard]] std::string add_namespaces(const std::string& top_ns, const std::string& sub_ns = "");

[[nodiscard]] std::string generate_internal_node_name(const std::string& prefix = "");

[[nodiscard]] rclcpp::Node::SharedPtr generate_internal_node(const std::string& prefix = "");

[[nodiscard]] std::string time_to_string(std::size_t len);

using ParameterDescriptor = rcl_interfaces::msg::ParameterDescriptor;

template <typename NodeT>
void declare_parameter_if_not_declared(NodeT node, const std::string& parameter_name,
                                       const rclcpp::ParameterValue& default_value,
                                       const ParameterDescriptor& parameter_descriptor = ParameterDescriptor()) {
    if (!node->has_parameter(parameter_name)) {
        node->declare_parameter(parameter_name, default_value, parameter_descriptor);
    }
}

template <typename NodeT>
void declare_parameter_if_not_declared(NodeT node, const std::string& parameter_name,
                                       const rclcpp::ParameterType& param_type,
                                       const ParameterDescriptor& parameter_descriptor = ParameterDescriptor()) {
    if (!node->has_parameter(parameter_name)) {
        node->declare_parameter(parameter_name, param_type, parameter_descriptor);
    }
}

template <typename ParameterT, typename NodeT>
ParameterT declare_or_get_parameter(NodeT node, const std::string& parameter_name,
                                    const rclcpp::ParameterType& param_type,
                                    const ParameterDescriptor& parameter_descriptor = ParameterDescriptor()) {
    if (node->has_parameter(parameter_name)) {
        return node->get_parameter(parameter_name).template get_value<ParameterT>();
    }

    auto parameter = node->declare_parameter(parameter_name, param_type, parameter_descriptor);
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
        std::string description = "Parameter " + parameter_name + " not in overrides";
        throw rclcpp::exceptions::InvalidParameterValueException(description.c_str());
    }
    return parameter.template get<ParameterT>();
}

using NodeParamInterfacePtr = rclcpp::node_interfaces::NodeParametersInterface::SharedPtr;

template <typename ParamType>
ParamType declare_or_get_parameter(const rclcpp::Logger& logger, NodeParamInterfacePtr param_interface,
                                   const std::string& parameter_name, const ParamType& default_value,
                                   bool warn_if_no_override = false, bool strict_param_loading = false,
                                   const ParameterDescriptor& parameter_descriptor = ParameterDescriptor()) {
    if (param_interface->has_parameter(parameter_name)) {
        rclcpp::Parameter param(parameter_name, default_value);
        param_interface->get_parameter(parameter_name, param);
        return param.get_value<ParamType>();
    }

    auto return_value =
        param_interface->declare_parameter(parameter_name, rclcpp::ParameterValue{default_value}, parameter_descriptor)
            .get<ParamType>();

    const bool no_param_override = param_interface->get_parameter_overrides().find(parameter_name) ==
                                   param_interface->get_parameter_overrides().end();

    if (no_param_override) {
        if (warn_if_no_override) {
            RCLCPP_WARN_STREAM(logger,
                               "Failed to get param " << parameter_name << " from overrides, using default value.");
        }
        if (strict_param_loading) {
            std::string description =
                "Parameter " + parameter_name + " not in overrides and strict_param_loading is True";
            throw rclcpp::exceptions::InvalidParameterValueException(description.c_str());
        }
    }

    return return_value;
}

template <typename ParamType, typename NodeT>
ParamType declare_or_get_parameter(NodeT node, const std::string& parameter_name, const ParamType& default_value,
                                   const ParameterDescriptor& parameter_descriptor = ParameterDescriptor()) {
    declare_parameter_if_not_declared(node, "warn_on_missing_params", rclcpp::ParameterValue(false));
    bool warn_if_no_override{false};
    node->get_parameter("warn_on_missing_params", warn_if_no_override);

    declare_parameter_if_not_declared(node, "strict_param_loading", rclcpp::ParameterValue(false));
    bool strict_param_loading{false};
    node->get_parameter("strict_param_loading", strict_param_loading);

    return declare_or_get_parameter(node->get_logger(), node->get_node_parameters_interface(), parameter_name,
                                    default_value, warn_if_no_override, strict_param_loading, parameter_descriptor);
}

template <typename NodeT> std::string get_plugin_type_param(NodeT node, const std::string& plugin_name) {
    declare_parameter_if_not_declared(node, plugin_name + ".plugin", rclcpp::PARAMETER_STRING);

    std::string plugin_type;
    try {
        if (!node->get_parameter(plugin_name + ".plugin", plugin_type)) {
            RCLCPP_FATAL(node->get_logger(), "Can not get 'plugin' param value for %s", plugin_name.c_str());
            throw pluginlib::PluginlibException("No 'plugin' param for param ns!");
        }
    } catch (rclcpp::exceptions::ParameterUninitializedException&) {
        RCLCPP_FATAL(node->get_logger(), "'plugin' param not defined for %s", plugin_name.c_str());
        throw pluginlib::PluginlibException("No 'plugin' param for param ns!");
    }

    return plugin_type;
}

void setSoftRealTimePriority();

} // namespace nav3d_util
