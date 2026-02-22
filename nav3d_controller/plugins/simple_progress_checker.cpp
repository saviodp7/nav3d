#include "nav3d_controller/plugins/simple_progress_checker.hpp"

#include <cmath>
#include <string>
#include <vector>
#include <functional>

#include "pluginlib/class_list_macros.hpp"
#include "nav3d_util/node_utils.hpp"
#include "tf2/utils.h"

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav3d_controller
{

static geometry_msgs::msg::Pose2D toPose2D(const geometry_msgs::msg::Pose & pose)
{
  geometry_msgs::msg::Pose2D out;
  out.x = pose.position.x;
  out.y = pose.position.y;
  out.theta = tf2::getYaw(pose.orientation);
  return out;
}

void SimpleProgressChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name)
{
  plugin_name_ = plugin_name;
  auto node = parent.lock();

  clock_ = node->get_clock();

  nav3d_util::declare_parameter_if_not_declared(
    node, plugin_name + ".required_movement_radius", rclcpp::ParameterValue(0.5));
  nav3d_util::declare_parameter_if_not_declared(
    node, plugin_name + ".movement_time_allowance", rclcpp::ParameterValue(10.0));

  node->get_parameter_or(plugin_name + ".required_movement_radius", radius_, 0.5);

  double time_allowance_param = 0.0;
  node->get_parameter_or(plugin_name + ".movement_time_allowance", time_allowance_param, 10.0);
  time_allowance_ = rclcpp::Duration::from_seconds(time_allowance_param);

  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&SimpleProgressChecker::dynamicParametersCallback, this, _1));
}

bool SimpleProgressChecker::check(geometry_msgs::msg::PoseStamped & current_pose)
{
  const auto current_pose2d = toPose2D(current_pose.pose);

  if (!baseline_pose_set_ || isRobotMovedEnough(current_pose2d)) {
    resetBaselinePose(current_pose2d);
    return true;
  }

  return (clock_->now() - baseline_time_) <= time_allowance_;
}

void SimpleProgressChecker::reset()
{
  baseline_pose_set_ = false;
}

void SimpleProgressChecker::resetBaselinePose(const geometry_msgs::msg::Pose2D & pose)
{
  baseline_pose_ = pose;
  baseline_time_ = clock_->now();
  baseline_pose_set_ = true;
}

bool SimpleProgressChecker::isRobotMovedEnough(const geometry_msgs::msg::Pose2D & pose)
{
  return pose_distance(pose, baseline_pose_) > radius_;
}

double SimpleProgressChecker::pose_distance(
  const geometry_msgs::msg::Pose2D & a,
  const geometry_msgs::msg::Pose2D & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

rcl_interfaces::msg::SetParametersResult
SimpleProgressChecker::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;

  for (auto & parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".required_movement_radius") {
        radius_ = parameter.as_double();
      } else if (name == plugin_name_ + ".movement_time_allowance") {
        time_allowance_ = rclcpp::Duration::from_seconds(parameter.as_double());
      }
    }
  }

  result.successful = true;
  return result;
}

}  // namespace nav3d_controller

PLUGINLIB_EXPORT_CLASS(nav3d_controller::SimpleProgressChecker, nav3d_core::ProgressChecker)