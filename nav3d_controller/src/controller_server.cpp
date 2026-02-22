#include "nav3d_controller/controller_server.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>
#include <functional>

#include "nav3d_core/controller_exceptions.hpp"
#include "nav3d_util/node_utils.hpp"
#include "rcl/time.h"
#include "tf2_ros/create_timer_ros.h"

using namespace std::chrono_literals;
using nav3d_util::declare_parameter_if_not_declared;

namespace
{

rclcpp::NodeOptions withAutoDeclaredParams(const rclcpp::NodeOptions & in)
{
  auto out = in;
  out.automatically_declare_parameters_from_overrides(true);
  return out;
}

}  // namespace

namespace nav3d_controller
{

ControllerServer::ControllerServer(const rclcpp::NodeOptions & options)
: nav3d_util::LifecycleNode("controller_server", "", withAutoDeclaredParams(options))
{
  declare_parameter_if_not_declared(this, "controller_frequency", rclcpp::ParameterValue(10.0));
  declare_parameter_if_not_declared(this, "waypoint_topic", rclcpp::ParameterValue(std::string("offboard/waypoint")));

  declare_parameter_if_not_declared(this, "global_frame", rclcpp::ParameterValue(std::string("map")));
  declare_parameter_if_not_declared(this, "robot_base_frame", rclcpp::ParameterValue(std::string("base_link")));
  declare_parameter_if_not_declared(this, "transform_tolerance", rclcpp::ParameterValue(0.1));

  declare_parameter_if_not_declared(this, "xyz_goal_tolerance", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(this, "yaw_goal_tolerance", rclcpp::ParameterValue(0.1));

  declare_parameter_if_not_declared(this, "speed_limit_topic", rclcpp::ParameterValue(std::string("speed_limit")));
  declare_parameter_if_not_declared(this, "default_speed_limit", rclcpp::ParameterValue(0.5));
}

ControllerServer::~ControllerServer() = default;

nav3d_util::CallbackReturn ControllerServer::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;

  controller_frequency_ = get_parameter("controller_frequency").as_double();
  waypoint_topic_ = get_parameter("waypoint_topic").as_string();

  global_frame_ = get_parameter("global_frame").as_string();
  robot_base_frame_ = get_parameter("robot_base_frame").as_string();
  transform_tolerance_ = get_parameter("transform_tolerance").as_double();

  xyz_goal_tolerance_ = get_parameter("xyz_goal_tolerance").as_double();
  yaw_goal_tolerance_ = get_parameter("yaw_goal_tolerance").as_double();

  default_speed_limit_ = get_parameter("default_speed_limit").as_double();
  speed_limit_ = default_speed_limit_;

  std::string speed_limit_topic = get_parameter("speed_limit_topic").as_string();

  waypoint_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(waypoint_topic_, rclcpp::QoS(10));

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(), get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

  try {
    rcl_action_server_options_t server_options = rcl_action_server_get_default_options();

    action_server_ = std::make_unique<ActionServer>(
      shared_from_this(),
      "follow_path",
      std::bind(&ControllerServer::computeControl, this),
      nullptr,
      std::chrono::milliseconds(500),
      true,
      server_options);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Error creating follow_path action server: %s", e.what());
    on_cleanup(state);
    return nav3d_util::CallbackReturn::FAILURE;
  }

  speed_limit_sub_ = create_subscription<nav3d_msgs::msg::SpeedLimit>(
    speed_limit_topic,
    rclcpp::QoS(10),
    std::bind(&ControllerServer::speedLimitCallback, this, std::placeholders::_1));

  current_path_.clear();
  current_waypoint_index_ = 0;

  return nav3d_util::CallbackReturn::SUCCESS;
}

nav3d_util::CallbackReturn ControllerServer::on_activate(const rclcpp_lifecycle::State &)
{
  action_server_->activate();
  createBond();
  return nav3d_util::CallbackReturn::SUCCESS;
}

nav3d_util::CallbackReturn ControllerServer::on_deactivate(const rclcpp_lifecycle::State &)
{
  action_server_->deactivate();
  destroyBond();
  return nav3d_util::CallbackReturn::SUCCESS;
}

nav3d_util::CallbackReturn ControllerServer::on_cleanup(const rclcpp_lifecycle::State &)
{
  action_server_.reset();
  waypoint_pub_.reset();
  speed_limit_sub_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();

  current_path_.clear();
  current_waypoint_index_ = 0;

  return nav3d_util::CallbackReturn::SUCCESS;
}

nav3d_util::CallbackReturn ControllerServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  return nav3d_util::CallbackReturn::SUCCESS;
}

void ControllerServer::computeControl()
{
  try {
    auto goal = action_server_->get_current_goal();
    if (!goal) {
      action_server_->terminate_all();
      return;
    }

    generateTimedTrajectory(goal->path);

    last_valid_cmd_time_ = now();
    rclcpp::WallRate loop_rate(controller_frequency_);

    while (rclcpp::ok()) {
      const auto start_time = now();

      if (!action_server_ || !action_server_->is_server_active()) {
        return;
      }

      if (action_server_->is_cancel_requested()) {
        action_server_->terminate_all();
        return;
      }

      if (action_server_->is_preempt_requested()) {
        action_server_->accept_pending_goal();
        goal = action_server_->get_current_goal();
        if (!goal) {
          action_server_->terminate_all();
          return;
        }

        current_path_.clear();
        current_waypoint_index_ = 0;
        generateTimedTrajectory(goal->path);
        last_valid_cmd_time_ = now();
      }

      const auto wp = getCurrentWaypoint();
      if (wp.has_value()) {
        auto msg = *wp;
        msg.header.stamp = now();
        waypoint_pub_->publish(msg);
      }

      if (isGoalReached()) {
        current_path_.clear();
        current_waypoint_index_ = 0;
        action_server_->succeeded_current();
        return;
      }

      const auto cycle = now() - start_time;
      if (!loop_rate.sleep()) {
        const double hz = cycle.seconds() > 0.0 ? 1.0 / cycle.seconds() : 0.0;
        RCLCPP_WARN(
          get_logger(),
          "Control loop missed rate %.4f Hz (current ~ %.4f Hz).",
          controller_frequency_, hz);
      }
    }

    action_server_->terminate_all();
    return;

  } catch (nav3d_core::ControllerTFError & e) {
    auto result = std::make_shared<Action::Result>();
    result->error_code = Action::Result::TF_ERROR;
    result->error_msg = e.what();
    action_server_->terminate_current(result);
    return;
  } catch (nav3d_core::PatienceExceeded & e) {
    auto result = std::make_shared<Action::Result>();
    result->error_code = Action::Result::PATIENCE_EXCEEDED;
    result->error_msg = e.what();
    action_server_->terminate_current(result);
    return;
  } catch (nav3d_core::InvalidPath & e) {
    auto result = std::make_shared<Action::Result>();
    result->error_code = Action::Result::INVALID_PATH;
    result->error_msg = e.what();
    action_server_->terminate_current(result);
    return;
  } catch (nav3d_core::ControllerTimedOut & e) {
    auto result = std::make_shared<Action::Result>();
    result->error_code = Action::Result::CONTROLLER_TIMED_OUT;
    result->error_msg = e.what();
    action_server_->terminate_current(result);
    return;
  } catch (nav3d_core::ControllerException & e) {
    auto result = std::make_shared<Action::Result>();
    result->error_code = Action::Result::UNKNOWN;
    result->error_msg = e.what();
    action_server_->terminate_current(result);
    return;
  } catch (std::exception & e) {
    auto result = std::make_shared<Action::Result>();
    result->error_code = Action::Result::UNKNOWN;
    result->error_msg = e.what();
    action_server_->terminate_current(result);
    return;
  }
}

void ControllerServer::generateTimedTrajectory(const nav_msgs::msg::Path & path)
{
  if (path.poses.empty()) {
    throw nav3d_core::InvalidPath("Path is empty.");
  }

  current_path_.clear();
  current_waypoint_index_ = 0;
  current_path_.reserve(path.poses.size());

  auto poses = path.poses;

  const rclcpp::Time start_time = now() + rclcpp::Duration::from_seconds(0.1);
  double accumulated_time = 0.0;
  const double v = std::max(1e-3, getSpeedLimit());

  poses[0].header.stamp = start_time;
  current_path_.emplace_back(start_time, poses[0]);

  for (size_t i = 1; i < poses.size(); ++i) {
    const auto & p0 = poses[i - 1].pose.position;
    const auto & p1 = poses[i].pose.position;

    const double dx = p1.x - p0.x;
    const double dy = p1.y - p0.y;
    const double dz = p1.z - p0.z;
    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    double dt = dist / v;
    dt = std::max(dt, 0.01);
    accumulated_time += dt;

    const rclcpp::Time t = start_time + rclcpp::Duration::from_seconds(accumulated_time);
    poses[i].header.stamp = t;
    current_path_.emplace_back(t, poses[i]);
  }
}

std::optional<geometry_msgs::msg::PoseStamped> ControllerServer::getCurrentWaypoint()
{
  if (current_path_.empty()) {
    return std::nullopt;
  }

  const rclcpp::Time tnow = now();

  while (current_waypoint_index_ < current_path_.size()) {
    const auto & item = current_path_[current_waypoint_index_];
    if (tnow < item.first) {
      return item.second;
    }
    ++current_waypoint_index_;
  }

  return current_path_.back().second;
}

bool ControllerServer::isGoalReached()
{
  if (!tf_buffer_) {
    return false;
  }
  if (current_path_.empty()) {
    return false;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav3d_util::getCurrentPose(
        current_pose, *tf_buffer_, global_frame_, robot_base_frame_, transform_tolerance_))
  {
    return false;
  }

  const auto & goal_pose = current_path_.back().second.pose;
  const auto & cur_pose = current_pose.pose;

  const double dx = cur_pose.position.x - goal_pose.position.x;
  const double dy = cur_pose.position.y - goal_pose.position.y;
  const double dz = cur_pose.position.z - goal_pose.position.z;
  const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

  if (dist > xyz_goal_tolerance_) {
    return false;
  }

  const double cur_yaw = tf2::getYaw(cur_pose.orientation);
  const double goal_yaw = tf2::getYaw(goal_pose.orientation);
  const double dyaw = angles::shortest_angular_distance(cur_yaw, goal_yaw);

  return std::fabs(dyaw) <= yaw_goal_tolerance_;
}

void ControllerServer::speedLimitCallback(const nav3d_msgs::msg::SpeedLimit::SharedPtr msg)
{
  if (msg->speed_limit > 0.0) {
    speed_limit_ = msg->speed_limit;
  } else {
    speed_limit_ = default_speed_limit_;
  }
}

}  // namespace nav3d_controller

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(nav3d_controller::ControllerServer)