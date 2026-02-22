#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <functional>

#include "pluginlib/class_list_macros.hpp"

#include "nav3d_controller/plugins/stopped_goal_checker.hpp"
#include "nav3d_util/node_utils.hpp"

using std::fabs;
using std::hypot;

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav3d_controller
{

StoppedGoalChecker::StoppedGoalChecker()
: SimpleGoalChecker(),
  rot_stopped_velocity_(0.25),
  trans_stopped_velocity_(0.25)
{
}

void StoppedGoalChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name)
{
  plugin_name_ = plugin_name;
  SimpleGoalChecker::initialize(parent, plugin_name);

  auto node = parent.lock();

  nav3d_util::declare_parameter_if_not_declared(
    node, plugin_name + ".rot_stopped_velocity", rclcpp::ParameterValue(0.25));
  nav3d_util::declare_parameter_if_not_declared(
    node, plugin_name + ".trans_stopped_velocity", rclcpp::ParameterValue(0.25));

  node->get_parameter(plugin_name + ".rot_stopped_velocity", rot_stopped_velocity_);
  node->get_parameter(plugin_name + ".trans_stopped_velocity", trans_stopped_velocity_);

  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&StoppedGoalChecker::dynamicParametersCallback, this, _1));
}

bool StoppedGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose,
  const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist & velocity)
{
  const bool in_goal_window = SimpleGoalChecker::isGoalReached(query_pose, goal_pose, velocity);
  if (!in_goal_window) {
    return false;
  }

  return fabs(velocity.angular.z) <= rot_stopped_velocity_ &&
         hypot(velocity.linear.x, velocity.linear.y) <= trans_stopped_velocity_;
}

bool StoppedGoalChecker::getTolerances(
  geometry_msgs::msg::Pose & pose_tolerance,
  geometry_msgs::msg::Twist & vel_tolerance)
{
  const double invalid_field = std::numeric_limits<double>::lowest();

  const bool ok = SimpleGoalChecker::getTolerances(pose_tolerance, vel_tolerance);

  vel_tolerance.linear.x = trans_stopped_velocity_;
  vel_tolerance.linear.y = trans_stopped_velocity_;
  vel_tolerance.linear.z = invalid_field;

  vel_tolerance.angular.x = invalid_field;
  vel_tolerance.angular.y = invalid_field;
  vel_tolerance.angular.z = rot_stopped_velocity_;

  return ok;
}

rcl_interfaces::msg::SetParametersResult
StoppedGoalChecker::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;

  for (auto & parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();

    if (param_name.find(plugin_name_ + ".") != 0) {
      continue;
    }

    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == plugin_name_ + ".rot_stopped_velocity") {
        rot_stopped_velocity_ = parameter.as_double();
      } else if (param_name == plugin_name_ + ".trans_stopped_velocity") {
        trans_stopped_velocity_ = parameter.as_double();
      }
    }
  }

  result.successful = true;
  return result;
}

}  // namespace nav3d_controller

PLUGINLIB_EXPORT_CLASS(nav3d_controller::StoppedGoalChecker, nav3d_core::GoalChecker)