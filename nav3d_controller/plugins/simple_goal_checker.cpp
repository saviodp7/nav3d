#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <functional>

#include "angles/angles.h"
#include "pluginlib/class_list_macros.hpp"

#include "nav3d_controller/plugins/simple_goal_checker.hpp"
#include "nav3d_util/geometry_utils.hpp"
#include "nav3d_util/node_utils.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.hpp"
#pragma GCC diagnostic pop

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav3d_controller
{

SimpleGoalChecker::SimpleGoalChecker()
: xyz_goal_tolerance_(0.05),
  yaw_goal_tolerance_(0.05),
  stateful_(true),
  check_xyz_(true),
  xyz_goal_tolerance_sq_(xyz_goal_tolerance_ * xyz_goal_tolerance_)
{
}

void SimpleGoalChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name)
{
  plugin_name_ = plugin_name;
  auto node = parent.lock();

  nav3d_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".xyz_goal_tolerance", rclcpp::ParameterValue(0.25));
  nav3d_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".yaw_goal_tolerance", rclcpp::ParameterValue(0.25));
  nav3d_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".stateful", rclcpp::ParameterValue(true));

  node->get_parameter(plugin_name + ".xyz_goal_tolerance", xyz_goal_tolerance_);
  node->get_parameter(plugin_name + ".yaw_goal_tolerance", yaw_goal_tolerance_);
  node->get_parameter(plugin_name + ".stateful", stateful_);

  xyz_goal_tolerance_sq_ = xyz_goal_tolerance_ * xyz_goal_tolerance_;

  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&SimpleGoalChecker::dynamicParametersCallback, this, _1));
}

void SimpleGoalChecker::reset()
{
  check_xyz_ = true;
}

bool SimpleGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose,
  const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist &)
{
  if (check_xyz_) {
    const double dx = query_pose.position.x - goal_pose.position.x;
    const double dy = query_pose.position.y - goal_pose.position.y;
    const double dz = query_pose.position.z - goal_pose.position.z;

    if (dx * dx + dy * dy + dz * dz > xyz_goal_tolerance_sq_) {
      return false;
    }

    if (stateful_) {
      check_xyz_ = false;
    }
  }

  const double dyaw = angles::shortest_angular_distance(
    tf2::getYaw(query_pose.orientation),
    tf2::getYaw(goal_pose.orientation));

  return std::fabs(dyaw) <= yaw_goal_tolerance_;
}

bool SimpleGoalChecker::getTolerances(
  geometry_msgs::msg::Pose & pose_tolerance,
  geometry_msgs::msg::Twist & vel_tolerance)
{
  const double invalid_field = std::numeric_limits<double>::lowest();

  pose_tolerance.position.x = xyz_goal_tolerance_;
  pose_tolerance.position.y = xyz_goal_tolerance_;
  pose_tolerance.position.z = xyz_goal_tolerance_;
  pose_tolerance.orientation =
    nav3d_util::geometry_utils::orientationAroundZAxis(yaw_goal_tolerance_);

  vel_tolerance.linear.x = invalid_field;
  vel_tolerance.linear.y = invalid_field;
  vel_tolerance.linear.z = invalid_field;

  vel_tolerance.angular.x = invalid_field;
  vel_tolerance.angular.y = invalid_field;
  vel_tolerance.angular.z = invalid_field;

  return true;
}

rcl_interfaces::msg::SetParametersResult
SimpleGoalChecker::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;

  for (auto & parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();

    if (param_name.find(plugin_name_ + ".") != 0) {
      continue;
    }

    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == plugin_name_ + ".xyz_goal_tolerance") {
        xyz_goal_tolerance_ = parameter.as_double();
        xyz_goal_tolerance_sq_ = xyz_goal_tolerance_ * xyz_goal_tolerance_;
      } else if (param_name == plugin_name_ + ".yaw_goal_tolerance") {
        yaw_goal_tolerance_ = parameter.as_double();
      }
    } else if (param_type == ParameterType::PARAMETER_BOOL) {
      if (param_name == plugin_name_ + ".stateful") {
        stateful_ = parameter.as_bool();
      }
    }
  }

  result.successful = true;
  return result;
}

}  // namespace nav3d_controller

PLUGINLIB_EXPORT_CLASS(nav3d_controller::SimpleGoalChecker, nav3d_core::GoalChecker)