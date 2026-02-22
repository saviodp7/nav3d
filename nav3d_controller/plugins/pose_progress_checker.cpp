#include "nav3d_controller/plugins/pose_progress_checker.hpp"

#include <cmath>
#include <string>
#include <vector>
#include <functional>

#include "angles/angles.h"
#include "pluginlib/class_list_macros.hpp"
#include "nav3d_util/node_utils.hpp"

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

void PoseProgressChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name)
{
  plugin_name_ = plugin_name;
  SimpleProgressChecker::initialize(parent, plugin_name);

  auto node = parent.lock();

  nav3d_util::declare_parameter_if_not_declared(
    node, plugin_name + ".required_movement_angle", rclcpp::ParameterValue(0.5));
  node->get_parameter_or(plugin_name + ".required_movement_angle", required_movement_angle_, 0.5);

  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&PoseProgressChecker::dynamicParametersCallback, this, _1));
}

bool PoseProgressChecker::check(geometry_msgs::msg::PoseStamped & current_pose)
{
  const auto pose2d = toPose2D(current_pose.pose);

  if (!baseline_pose_set_ || isRobotMovedEnough(pose2d)) {
    resetBaselinePose(pose2d);
    return true;
  }

  return clock_->now() - baseline_time_ <= time_allowance_;
}

bool PoseProgressChecker::isRobotMovedEnough(const geometry_msgs::msg::Pose2D & pose)
{
  return pose_distance(pose, baseline_pose_) > radius_ ||
         poseAngleDistance(pose, baseline_pose_) > required_movement_angle_;
}

double PoseProgressChecker::poseAngleDistance(
  const geometry_msgs::msg::Pose2D & a,
  const geometry_msgs::msg::Pose2D & b)
{
  return std::fabs(angles::shortest_angular_distance(a.theta, b.theta));
}

rcl_interfaces::msg::SetParametersResult
PoseProgressChecker::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;

  for (auto & parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();

    if (param_name.find(plugin_name_ + ".") != 0) {
      continue;
    }

    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == plugin_name_ + ".required_movement_angle") {
        required_movement_angle_ = parameter.as_double();
      }
    }
  }

  result.successful = true;
  return result;
}

}  // namespace nav3d_controller

PLUGINLIB_EXPORT_CLASS(nav3d_controller::PoseProgressChecker, nav3d_core::ProgressChecker)