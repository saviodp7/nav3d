#include "nav3d_px4_bridge/offboard_control.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <tf2/utils.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace nav3d_px4_bridge {

static rclcpp::QoS px4Qos() {
    return rclcpp::QoS(1)
        .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE)
        .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
}

OffboardControl::OffboardControl() : rclcpp::Node("offboard_control"), logger_(get_logger()) {
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 50.0);
    waypoint_timeout_s_ = declare_parameter<double>("waypoint_timeout_s", 1.0);
    offboard_warmup_s_ = declare_parameter<double>("offboard_warmup_s", 1.0);
    offboard_command_retry_s_ = declare_parameter<double>("offboard_command_retry_s", 1.0);
    offset_alpha_ = declare_parameter<double>("offset_alpha", 0.01);

    takeoff_altitude_ = declare_parameter<double>("takeoff_altitude", 1.0);
    altitude_tolerance_ = declare_parameter<double>("altitude_tolerance", 0.1);

    odom_topic_ = declare_parameter<std::string>("odom_topic", odom_topic_);
    waypoint_topic_ = declare_parameter<std::string>("waypoint_topic", waypoint_topic_);

    fmu_in_offboard_control_mode_ =
        declare_parameter<std::string>("fmu_in_offboard_control_mode", fmu_in_offboard_control_mode_);
    fmu_in_traj_setpoint_ = declare_parameter<std::string>("fmu_in_traj_setpoint", fmu_in_traj_setpoint_);
    fmu_in_vehicle_command_ = declare_parameter<std::string>("fmu_in_vehicle_command", fmu_in_vehicle_command_);

    fmu_out_vehicle_status_ = declare_parameter<std::string>("fmu_out_vehicle_status", fmu_out_vehicle_status_);
    fmu_out_vehicle_local_position_ =
        declare_parameter<std::string>("fmu_out_vehicle_local_position", fmu_out_vehicle_local_position_);

    offboard_control_mode_pub_ =
        create_publisher<px4_msgs::msg::OffboardControlMode>(fmu_in_offboard_control_mode_, px4Qos());
    traj_setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(fmu_in_traj_setpoint_, px4Qos());
    vehicle_cmd_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(fmu_in_vehicle_command_, px4Qos());

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(odom_topic_, 10,
                                                             std::bind(&OffboardControl::odomCallback, this, _1));

    vehicle_local_pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        fmu_out_vehicle_local_position_, px4Qos(), std::bind(&OffboardControl::vehicleLocalPositionCallback, this, _1));

    vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
        fmu_out_vehicle_status_, px4Qos(), std::bind(&OffboardControl::vehicleStatusCallback, this, _1));

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    waypoint_sub_ = create_subscription<nav3d_msgs::msg::TrajectoryPoint>(
        waypoint_topic_, qos, std::bind(&OffboardControl::waypointCallback, this, _1));

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / std::max(1e-3, control_rate_hz_)));
    timer_ = create_wall_timer(period, std::bind(&OffboardControl::timerCallback, this));
}

void OffboardControl::vehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
    current_local_pos_ = msg;
}

void OffboardControl::vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
    vehicle_status_ = *msg;
}

void OffboardControl::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_odom_ = msg;
    updateOffset();
}

void OffboardControl::updateOffset() {
    if (!current_odom_ || !current_local_pos_) {
        return;
    }

    const double ned_from_odom_x = current_odom_->pose.pose.position.y;
    const double ned_from_odom_y = current_odom_->pose.pose.position.x;
    const double ned_from_odom_z = -current_odom_->pose.pose.position.z;

    const double ned_px4_x = current_local_pos_->x;
    const double ned_px4_y = current_local_pos_->y;
    const double ned_px4_z = current_local_pos_->z;

    const double ex = ned_px4_x - ned_from_odom_x;
    const double ey = ned_px4_y - ned_from_odom_y;
    const double ez = ned_px4_z - ned_from_odom_z;

    const double a = std::clamp(offset_alpha_, 0.0, 1.0);
    odom_to_px4_offset_.x = a * ex + (1.0 - a) * odom_to_px4_offset_.x;
    odom_to_px4_offset_.y = a * ey + (1.0 - a) * odom_to_px4_offset_.y;
    odom_to_px4_offset_.z = a * ez + (1.0 - a) * odom_to_px4_offset_.z;
}

geometry_msgs::msg::Pose OffboardControl::enuToNedPose(const geometry_msgs::msg::Pose& enu_pose) const {
    geometry_msgs::msg::Pose ned_pose;
    ned_pose.position.x = enu_pose.position.y;
    ned_pose.position.y = enu_pose.position.x;
    ned_pose.position.z = -enu_pose.position.z;

    tf2::Quaternion enu_q;
    tf2::fromMsg(enu_pose.orientation, enu_q);

    double r, p, y;
    tf2::Matrix3x3(enu_q).getRPY(r, p, y);

    double yaw_ned = -y + M_PI / 2.0;
    while (yaw_ned > M_PI)
        yaw_ned -= 2.0 * M_PI;
    while (yaw_ned < -M_PI)
        yaw_ned += 2.0 * M_PI;

    tf2::Quaternion ned_q;
    ned_q.setRPY(r, p, yaw_ned);
    ned_pose.orientation = tf2::toMsg(ned_q);

    return ned_pose;
}

void OffboardControl::fillPositionVelocityFields(px4_msgs::msg::TrajectorySetpoint& msg) {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    msg.acceleration[0] = nan;
    msg.acceleration[1] = nan;
    msg.acceleration[2] = nan;
    msg.jerk[0] = nan;
    msg.jerk[1] = nan;
    msg.jerk[2] = nan;
    msg.yawspeed = nan;
}

void OffboardControl::waypointCallback(const nav3d_msgs::msg::TrajectoryPoint::SharedPtr msg) {
    last_valid_setpoint_enu_ = EnuSetpoint{msg->pose, msg->velocity};
    last_waypoint_time_ = get_clock()->now();

    auto ned_pose = enuToNedPose(msg->pose);
    ned_pose.position.x += odom_to_px4_offset_.x;
    ned_pose.position.y += odom_to_px4_offset_.y;
    ned_pose.position.z += odom_to_px4_offset_.z;

    const double vx_enu = msg->velocity.linear.x;
    const double vy_enu = msg->velocity.linear.y;
    const double vz_enu = msg->velocity.linear.z;

    const float vx_ned = static_cast<float>(vy_enu);
    const float vy_ned = static_cast<float>(vx_enu);
    const float vz_ned = static_cast<float>(-vz_enu);

    current_waypoint_ned_ = std::make_shared<px4_msgs::msg::TrajectorySetpoint>();
    current_waypoint_ned_->position[0] = static_cast<float>(ned_pose.position.x);
    current_waypoint_ned_->position[1] = static_cast<float>(ned_pose.position.y);
    current_waypoint_ned_->position[2] = static_cast<float>(ned_pose.position.z);

    current_waypoint_ned_->velocity[0] = vx_ned;
    current_waypoint_ned_->velocity[1] = vy_ned;
    current_waypoint_ned_->velocity[2] = vz_ned;

    const double yaw_ned = tf2::getYaw(ned_pose.orientation);
    current_waypoint_ned_->yaw = static_cast<float>(yaw_ned);

    fillPositionVelocityFields(*current_waypoint_ned_);
}

void OffboardControl::publishOffboardControlHeartbeat() {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = true;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_pub_->publish(msg);
}

void OffboardControl::publishVehicleCommand(uint16_t command, float param1, float param2, float param3, float param4,
                                            float param5, float param6, float param7) {
    px4_msgs::msg::VehicleCommand msg{};
    msg.command = command;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.param3 = param3;
    msg.param4 = param4;
    msg.param5 = param5;
    msg.param6 = param6;
    msg.param7 = param7;

    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;

    vehicle_cmd_pub_->publish(msg);
}

void OffboardControl::arm() {
    publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
}

void OffboardControl::disarm() {
    publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
}

void OffboardControl::engageOffboardMode() {
    publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
}

void OffboardControl::publishTakeoffSetpoint() {
    if (!current_odom_) {
        return;
    }

    if (!takeoff_start_enu_) {
        takeoff_start_enu_.emplace();
        takeoff_start_enu_->pose.position.x = current_odom_->pose.pose.position.x;
        takeoff_start_enu_->pose.position.y = current_odom_->pose.pose.position.y;
        takeoff_start_enu_->pose.position.z = current_odom_->pose.pose.position.z;
        takeoff_start_enu_->pose.orientation = current_odom_->pose.pose.orientation;

        auto& q = takeoff_start_enu_->pose.orientation;
        const double q_norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
        if (q_norm > 1e-6) {
            q.x /= q_norm;
            q.y /= q_norm;
            q.z /= q_norm;
            q.w /= q_norm;
        } else {
            tf2::Quaternion identity;
            identity.setRPY(0.0, 0.0, 0.0);
            q = tf2::toMsg(identity);
        }

        takeoff_target_altitude_ = takeoff_start_enu_->pose.position.z + takeoff_altitude_;
    }

    geometry_msgs::msg::Pose enu_pose = takeoff_start_enu_->pose;
    enu_pose.position.z = takeoff_target_altitude_.value();
    last_valid_setpoint_enu_ = EnuSetpoint{enu_pose, geometry_msgs::msg::Twist{}};

    auto ned_pose = enuToNedPose(enu_pose);
    ned_pose.position.x += odom_to_px4_offset_.x;
    ned_pose.position.y += odom_to_px4_offset_.y;
    ned_pose.position.z += odom_to_px4_offset_.z;

    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.position[0] = static_cast<float>(ned_pose.position.x);
    msg.position[1] = static_cast<float>(ned_pose.position.y);
    msg.position[2] = static_cast<float>(ned_pose.position.z);

    msg.velocity[0] = 0.0f;
    msg.velocity[1] = 0.0f;
    msg.velocity[2] = 0.0f;

    msg.yaw = static_cast<float>(tf2::getYaw(ned_pose.orientation));
    fillPositionVelocityFields(msg);

    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    traj_setpoint_pub_->publish(msg);
}

bool OffboardControl::isTakeoffComplete() {
    if (!current_odom_ || !takeoff_target_altitude_) {
        return false;
    }

    const double curr_alt = current_odom_->pose.pose.position.z;
    const double diff = std::fabs(curr_alt - takeoff_target_altitude_.value());

    if (diff < altitude_tolerance_ && !takeoff_completed_) {
        takeoff_completed_ = true;
    }

    return takeoff_completed_;
}

void OffboardControl::publishHoverSetpoint() {
    if (!last_valid_setpoint_enu_) {
        return;
    }

    auto ned_pose = enuToNedPose(last_valid_setpoint_enu_->pose);
    ned_pose.position.x += odom_to_px4_offset_.x;
    ned_pose.position.y += odom_to_px4_offset_.y;
    ned_pose.position.z += odom_to_px4_offset_.z;

    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.position[0] = static_cast<float>(ned_pose.position.x);
    msg.position[1] = static_cast<float>(ned_pose.position.y);
    msg.position[2] = static_cast<float>(ned_pose.position.z);

    msg.velocity[0] = 0.0f;
    msg.velocity[1] = 0.0f;
    msg.velocity[2] = 0.0f;

    msg.yaw = static_cast<float>(tf2::getYaw(ned_pose.orientation));
    fillPositionVelocityFields(msg);

    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    traj_setpoint_pub_->publish(msg);
}

void OffboardControl::timerCallback() {
    const auto now = get_clock()->now();

    publishOffboardControlHeartbeat();

    if (!current_odom_) {
        return;
    }

    const bool has_fresh_waypoint =
        current_waypoint_ned_ && (get_clock()->now() - last_waypoint_time_).seconds() < waypoint_timeout_s_;

    if (has_fresh_waypoint) {
        current_waypoint_ned_->timestamp = get_clock()->now().nanoseconds() / 1000;
        traj_setpoint_pub_->publish(*current_waypoint_ned_);
    } else if (isTakeoffComplete()) {
        publishHoverSetpoint();
    } else {
        publishTakeoffSetpoint();
    }

    const auto warmup_count =
        static_cast<uint64_t>(std::ceil(std::max(0.0, offboard_warmup_s_) * std::max(1e-3, control_rate_hz_)));
    const uint64_t required_setpoints = std::max<uint64_t>(1, warmup_count);

    if (offboard_setpoint_counter_ < required_setpoints) {
        offboard_setpoint_counter_++;
        return;
    }

    const bool is_offboard = vehicle_status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
    const bool is_armed = vehicle_status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    const double retry_period_s = std::max(0.1, offboard_command_retry_s_);
    const bool retry_due = !offboard_command_sent_ || (now - last_offboard_command_time_).seconds() >= retry_period_s;

    if ((!is_offboard || !is_armed) && retry_due) {
        engageOffboardMode();
        arm();
        offboard_command_sent_ = true;
        last_offboard_command_time_ = now;
    }
}

} // namespace nav3d_px4_bridge
