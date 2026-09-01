#include "nav3d_util/lifecycle_node.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "nav3d_util/node_utils.hpp"

#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace nav3d_util {

LifecycleNode::LifecycleNode(const std::string& node_name, const std::string& ns, const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode(node_name, ns, options) {
    declare_parameter(bond::msg::Constants::DISABLE_HEARTBEAT_TIMEOUT_PARAM, true);
    set_parameter(rclcpp::Parameter(bond::msg::Constants::DISABLE_HEARTBEAT_TIMEOUT_PARAM, true));

    nav3d_util::declare_parameter_if_not_declared(this, "bond_heartbeat_period", rclcpp::ParameterValue(0.1));
    get_parameter("bond_heartbeat_period", bond_heartbeat_period);

    bool autostart_node = false;
    nav3d_util::declare_parameter_if_not_declared(this, "autostart_node", rclcpp::ParameterValue(false));
    get_parameter("autostart_node", autostart_node);
    if (autostart_node) {
        autostart();
    }

    printLifecycleNodeNotification();
    register_rcl_preshutdown_callback();
}

LifecycleNode::~LifecycleNode() {
    RCLCPP_INFO(get_logger(), "Destroying");

    runCleanups();

    if (rcl_preshutdown_cb_handle_) {
        auto context = get_node_base_interface()->get_context();
        context->remove_pre_shutdown_callback(*rcl_preshutdown_cb_handle_);
        rcl_preshutdown_cb_handle_.reset();
    }
}

void LifecycleNode::createBond() {
    if (bond_heartbeat_period <= 0.0) {
        return;
    }

    RCLCPP_INFO(get_logger(), "Creating bond (%s) to lifecycle manager.", get_name());

    bond_ = std::make_unique<bond::Bond>(std::string("bond"), get_name(), shared_from_this());

    bond_->setHeartbeatPeriod(bond_heartbeat_period);
    bond_->setHeartbeatTimeout(4.0);
    bond_->start();
}

void LifecycleNode::autostart() {
    using lifecycle_msgs::msg::State;
    autostart_timer_ = create_wall_timer(0s, [this]() -> void {
        autostart_timer_->cancel();
        RCLCPP_INFO(get_logger(), "Auto-starting node: %s", get_name());
        if (configure().id() != State::PRIMARY_STATE_INACTIVE) {
            RCLCPP_ERROR(get_logger(), "Auto-starting node %s failed to configure!", get_name());
            return;
        }
        if (activate().id() != State::PRIMARY_STATE_ACTIVE) {
            RCLCPP_ERROR(get_logger(), "Auto-starting node %s failed to activate!", get_name());
        }
    });
}

void LifecycleNode::runCleanups() {
    if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        deactivate();
    }

    if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
        cleanup();
    }
}

void LifecycleNode::on_rcl_preshutdown() {
    RCLCPP_INFO(get_logger(), "Running Nav3D LifecycleNode rcl preshutdown (%s)", get_name());

    runCleanups();
    destroyBond();
}

void LifecycleNode::register_rcl_preshutdown_callback() {
    auto context = get_node_base_interface()->get_context();

    rcl_preshutdown_cb_handle_ = std::make_unique<rclcpp::PreShutdownCallbackHandle>(
        context->add_pre_shutdown_callback(std::bind(&LifecycleNode::on_rcl_preshutdown, this)));
}

void LifecycleNode::destroyBond() {
    if (bond_heartbeat_period <= 0.0) {
        return;
    }

    RCLCPP_INFO(get_logger(), "Destroying bond (%s) to lifecycle manager.", get_name());
    bond_.reset();
}

void LifecycleNode::printLifecycleNodeNotification() {
    RCLCPP_INFO(get_logger(),
                "\n\t%s lifecycle node launched. \n"
                "\tWaiting on external lifecycle transitions to activate\n"
                "\tSee https://design.ros2.org/articles/node_lifecycle.html for more information.",
                get_name());
}

} // namespace nav3d_util
