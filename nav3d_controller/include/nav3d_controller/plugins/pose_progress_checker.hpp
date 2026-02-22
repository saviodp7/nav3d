#pragma once

#include <string>
#include <vector>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "nav3d_controller/plugins/simple_progress_checker.hpp"

namespace nav3d_controller
{

class PoseProgressChecker : public SimpleProgressChecker
{
public:
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name) override;

  bool check(geometry_msgs::msg::PoseStamped & current_pose) override;

protected:
  bool isRobotMovedEnough(const geometry_msgs::msg::Pose2D & pose);

  static double poseAngleDistance(
    const geometry_msgs::msg::Pose2D & a,
    const geometry_msgs::msg::Pose2D & b);

  double required_movement_angle_{0.0};

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  std::string plugin_name_;

  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
};

}  // namespace nav3d_controller