#pragma once

#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav3d_core/behavior_tree_navigator.hpp"
#include "nav3d_msgs/action/navigate_to_pose.hpp"
#include "nav3d_util/geometry_utils.hpp"
#include "nav3d_util/robot_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav3d_util/odometry_utils.hpp"

namespace nav3d_bt_navigator
{

class NavigateToPoseNavigator
  : public nav3d_core::BehaviorTreeNavigator<nav3d_msgs::action::NavigateToPose>
{
public:
  using ActionT = nav3d_msgs::action::NavigateToPose;

  NavigateToPoseNavigator()
  : BehaviorTreeNavigator() {}

  bool configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr node,
    std::shared_ptr<nav3d_util::OdomSmoother> odom_smoother) override;

  bool cleanup() override;

  void onGoalPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr pose);

  std::string getName() override {return std::string("navigate_to_pose");}

  std::string getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr node) override;

protected:
  bool goalReceived(ActionT::Goal::ConstSharedPtr goal) override;

  void onLoop() override;

  void onPreempt(ActionT::Goal::ConstSharedPtr goal) override;

  void goalCompleted(
    typename ActionT::Result::SharedPtr result,
    const nav3d_behavior_tree::BtStatus final_bt_status) override;

  bool initializeGoalPose(ActionT::Goal::ConstSharedPtr goal);

  rclcpp::Time start_time_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp_action::Client<ActionT>::SharedPtr self_client_;

  std::string goal_blackboard_id_;
  std::string path_blackboard_id_;

  std::shared_ptr<nav3d_util::OdomSmoother> odom_smoother_;
};

}  // namespace nav3d_bt_navigator