#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <functional>

#include "pluginlib/class_list_macros.hpp"

#include "nav3d_controller/plugins/position_goal_checker.hpp"
#include "nav3d_util/node_utils.hpp"

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav3d_controller
{

PositionGoalChecker::PositionGoalChecker()
: xy_goal_tolerance_(0.25),
  xy_goal_tolerance_sq_(0.0625),
  stateful_(true),
  position_reached_(false)
{
}

void PositionGoalChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name)
{
  plugin_name_ = plugin_name;
  auto node = parent.lock();

  nav3d_util::declare_parameter_if_not_declared(
    node, plugin_name + ".xy_goal_tolerance", rclcpp::ParameterValue(0.25));
  nav3d_util::declare_parameter_if_not_declared(
    node, plugin_name + ".stateful", rclcpp::ParameterValue(true));

  node->get_parameter(plugin_name + ".xy_goal_tolerance", xy_goal_tolerance_);
  node->get_parameter(plugin_name + ".stateful", stateful_);

  xy_goal_tolerance_sq_ = xy_goal_tolerance_ * xy_goal_tolerance_;

  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&PositionGoalChecker::dynamicParametersCallback, this, _1));
}

void PositionGoalChecker::reset()
{
  position_reached_ = false;
}

bool PositionGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose,
  const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist &)
{
  if (stateful_ && position_reached_) {
    return true;
  }

  const double dx = query_pose.position.x - goal_pose.position.x;
  const double dy = query_pose.position.y - goal_pose.position.y;

  const bool reached = (dx * dx + dy * dy <= xy_goal_tolerance_sq_);

  if (stateful_ && reached) {
    position_reached_ = true;
  }

  return reached;
}

bool PositionGoalChecker::getTolerances(
  geometry_msgs::msg::Pose & pose_tolerance,
  geometry_msgs::msg::Twist & vel_tolerance)
{
  const double invalid_field = std::numeric_limits<double>::lowest();

  pose_tolerance.position.x = xy_goal_tolerance_;
  pose_tolerance.position.y = xy_goal_tolerance_;
  pose_tolerance.position.z = invalid_field;

  pose_tolerance.orientation.x = 0.0;
  pose_tolerance.orientation.y = 0.0;
  pose_tolerance.orientation.z = 0.0;
  pose_tolerance.orientation.w = 1.0;

  vel_tolerance.linear.x = invalid_field;
  vel_tolerance.linear.y = invalid_field;
  vel_tolerance.linear.z = invalid_field;

  vel_tolerance.angular.x = invalid_field;
  vel_tolerance.angular.y = invalid_field;
  vel_tolerance.angular.z = invalid_field;

  return true;
}

void PositionGoalChecker::setXYGoalTolerance(double tolerance)
{
  xy_goal_tolerance_ = tolerance;
  xy_goal_tolerance_sq_ = tolerance * tolerance;
}

rcl_interfaces::msg::SetParametersResult
PositionGoalChecker::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;

  for (auto & parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();

    if (param_name.find(plugin_name_ + ".") != 0) {
      continue;
    }

    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == plugin_name_ + ".xy_goal_tolerance") {
        xy_goal_tolerance_ = parameter.as_double();
        xy_goal_tolerance_sq_ = xy_goal_tolerance_ * xy_goal_tolerance_;
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

PLUGINLIB_EXPORT_CLASS(nav3d_controller::PositionGoalChecker, nav3d_core::GoalChecker)