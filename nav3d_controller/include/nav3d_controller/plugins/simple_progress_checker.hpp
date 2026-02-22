#pragma once

#include <string>
#include <vector>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "nav3d_core/progress_checker.hpp"

namespace nav3d_controller
{

class SimpleProgressChecker : public nav3d_core::ProgressChecker
{
public:
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name) override;

  bool check(geometry_msgs::msg::PoseStamped & current_pose) override;

  void reset() override;

protected:
  bool isRobotMovedEnough(const geometry_msgs::msg::Pose2D & pose);
  void resetBaselinePose(const geometry_msgs::msg::Pose2D & pose);

  static double pose_distance(
    const geometry_msgs::msg::Pose2D & a,
    const geometry_msgs::msg::Pose2D & b);

  rclcpp::Clock::SharedPtr clock_;

  double radius_{0.5};
  rclcpp::Duration time_allowance_{0, 0};

  geometry_msgs::msg::Pose2D baseline_pose_;
  rclcpp::Time baseline_time_;
  bool baseline_pose_set_{false};

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  std::string plugin_name_;

  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
};

}  // namespace nav3d_controller