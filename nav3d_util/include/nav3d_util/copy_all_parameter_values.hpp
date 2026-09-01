#pragma once

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/parameter.hpp"

#include <string>
#include <vector>

namespace nav3d_util {
template <typename NodeT1, typename NodeT2>
void copy_all_parameter_values(const NodeT1& source, const NodeT2& destination,
                               const bool override_existing_params = false) {
    using Parameters = std::vector<rclcpp::Parameter>;
    using Descriptions = std::vector<rcl_interfaces::msg::ParameterDescriptor>;

    auto source_params = source->get_node_parameters_interface();
    auto dest_params = destination->get_node_parameters_interface();
    rclcpp::Logger logger = destination->get_node_logging_interface()->get_logger();

    const std::vector<std::string> param_names = source_params->list_parameters({}, 0).names;
    const Parameters params = source_params->get_parameters(param_names);
    const Descriptions descriptions = source_params->describe_parameters(param_names);

    for (size_t idx = 0; idx < params.size(); ++idx) {
        const auto& p = params[idx];
        const auto& d = descriptions[idx];

        if (!dest_params->has_parameter(p.get_name())) {
            dest_params->declare_parameter(p.get_name(), p.get_parameter_value(), d);
            continue;
        }

        if (!override_existing_params) {
            continue;
        }

        try {
            const auto result = dest_params->set_parameters_atomically({p});
            if (!result.successful) {
                RCLCPP_WARN(logger, "Unable to set parameter (%s): %s!", p.get_name().c_str(), result.reason.c_str());
            }
        } catch (const rclcpp::exceptions::InvalidParameterTypeException& e) {
            RCLCPP_WARN(logger, "Unable to set parameter (%s): incompatible parameter type (%s)!", p.get_name().c_str(),
                        e.what());
        }
    }
}
} // namespace nav3d_util
