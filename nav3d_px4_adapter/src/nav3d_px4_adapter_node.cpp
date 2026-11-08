#include "nav3d_px4_adapter/nav3d_px4_adapter.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<nav3d_px4_adapter::Nav3dPx4Adapter>());
    rclcpp::shutdown();
    return 0;
}
