#pragma once

#include <atomic>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace nav3d_px4_bridge
{

class OffboardControl final : public rclcpp::Node
{
public:
  OffboardControl();

private:
  struct Offset
  {
    double x{0.0};
    double y{0.0};
    double z{0.0};
  };

  struct EnuSetpoint
  {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double yaw{0.0};
  };

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void vehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
  void waypointCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  void timerCallback();

  void updateOffset();
  geometry_msgs::msg::Pose enuToNedPose(const geometry_msgs::msg::Pose & enu_pose) const;

  void publishOffboardControlHeartbeat();
  void publishVehicleCommand(
    uint16_t command,
    float param1 = 0.0f, float param2 = 0.0f, float param3 = 0.0f,
    float param4 = 0.0f, float param5 = 0.0f, float param6 = 0.0f, float param7 = 0.0f);

  void arm();
  void disarm();
  void engageOffboardMode();

  void publishTakeoffSetpoint();
  void publishHoverSetpoint();

  bool isTakeoffComplete();

  void fillPositionOnlyFields(px4_msgs::msg::TrajectorySetpoint & msg);

  rclcpp::Logger logger_{rclcpp::get_logger("offboard_control")};

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr traj_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_cmd_pub_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_pos_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Odometry::SharedPtr current_odom_;
  px4_msgs::msg::VehicleLocalPosition::SharedPtr current_local_pos_;
  px4_msgs::msg::VehicleStatus vehicle_status_{};

  Offset odom_to_px4_offset_{};
  double offset_alpha_{0.01};

  std::shared_ptr<px4_msgs::msg::TrajectorySetpoint> current_waypoint_ned_;
  std::optional<EnuSetpoint> last_valid_setpoint_enu_;
  rclcpp::Time last_waypoint_time_{rclcpp::Time(0, 0, RCL_ROS_TIME)};
  double waypoint_timeout_s_{1.0};

  std::optional<EnuSetpoint> takeoff_start_enu_;
  std::optional<double> takeoff_target_altitude_;
  bool takeoff_completed_{false};
  double takeoff_altitude_{1.0};
  double altitude_tolerance_{0.1};

  std::atomic<uint64_t> offboard_setpoint_counter_{0};
  double control_rate_hz_{10.0};

  std::string odom_topic_{"/odom"};
  std::string waypoint_topic_{"/offboard/waypoint"};

  std::string fmu_in_offboard_control_mode_{"/fmu/in/offboard_control_mode"};
  std::string fmu_in_traj_setpoint_{"/fmu/in/trajectory_setpoint"};
  std::string fmu_in_vehicle_command_{"/fmu/in/vehicle_command"};

  std::string fmu_out_vehicle_status_{"/fmu/out/vehicle_status"};
  std::string fmu_out_vehicle_local_position_{"/fmu/out/vehicle_local_position"};
};

}  // namespace nav3d_px4_bridge