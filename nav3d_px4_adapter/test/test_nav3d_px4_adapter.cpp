#include "nav3d_px4_adapter/nav3d_px4_adapter.hpp"

#include <chrono>
#include <cstdint>
#include <gtest/gtest.h>
#include <memory>
#include <nav3d_msgs/msg/vehicle_state.hpp>
#include <optional>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

using namespace std::chrono_literals;

TEST(ControlState, ReflectsFreshOffboardStatus) {
    rclcpp::init(0, nullptr);

    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("vehicle_state_rate_hz", 40.0),
        rclcpp::Parameter("vehicle_status_timeout_s", 0.2),
        rclcpp::Parameter("fmu_out_vehicle_status", "/test_control_state/vehicle_status"),
        rclcpp::Parameter("vehicle_state_topic", "/test_control_state/state"),
    });

    auto adapter = std::make_shared<nav3d_px4_adapter::Nav3dPx4Adapter>(options);
    auto observer = std::make_shared<rclcpp::Node>("control_state_observer");
    auto status_pub = observer->create_publisher<px4_msgs::msg::VehicleStatus>("/test_control_state/vehicle_status",
                                                                               rclcpp::QoS(1).best_effort());

    std::optional<nav3d_msgs::msg::VehicleState> latest_state;
    auto state_sub = observer->create_subscription<nav3d_msgs::msg::VehicleState>(
        "/test_control_state/state", rclcpp::QoS(10),
        [&latest_state](const nav3d_msgs::msg::VehicleState::SharedPtr msg) { latest_state = *msg; });

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(adapter);
    executor.add_node(observer);

    const auto wait_for_state = [&](uint8_t expected, std::optional<uint8_t> nav_state = std::nullopt) {
        const auto deadline = std::chrono::steady_clock::now() + 2s;
        while (std::chrono::steady_clock::now() < deadline) {
            if (nav_state) {
                px4_msgs::msg::VehicleStatus status{};
                status.arming_state = px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED;
                status.nav_state = *nav_state;
                status_pub->publish(status);
            }
            executor.spin_some();
            if (latest_state && latest_state->control_state == expected) {
                return true;
            }
            std::this_thread::sleep_for(5ms);
        }
        return false;
    };

    EXPECT_TRUE(wait_for_state(nav3d_msgs::msg::VehicleState::CONTROL_UNKNOWN));
    EXPECT_TRUE(wait_for_state(nav3d_msgs::msg::VehicleState::PX4_CONTROL,
                               px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD));
    EXPECT_TRUE(wait_for_state(nav3d_msgs::msg::VehicleState::CONTROL_UNKNOWN,
                               px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_POSCTL));
    EXPECT_TRUE(wait_for_state(nav3d_msgs::msg::VehicleState::PX4_CONTROL,
                               px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD));
    EXPECT_TRUE(wait_for_state(nav3d_msgs::msg::VehicleState::CONTROL_UNKNOWN));
    EXPECT_TRUE(wait_for_state(nav3d_msgs::msg::VehicleState::PX4_CONTROL,
                               px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD));

    executor.remove_node(observer);
    executor.remove_node(adapter);
    state_sub.reset();
    status_pub.reset();
    observer.reset();
    adapter.reset();
    rclcpp::shutdown();
}
