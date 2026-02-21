#pragma once

#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav3d_core/behavior_tree_navigator.hpp"
#include "nav3d_msgs/action/navigate_through_poses.hpp"
#include "nav3d_msgs/msg/waypoint_status.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav3d_util/robot_utils.hpp"
#include "nav3d_util/geometry_utils.hpp"
#include "nav3d_util/odometry_utils.hpp"

namespace nav3d_bt_navigator
{

class NavigateThroughPosesNavigator
  : public nav3d_core::BehaviorTreeNavigator<nav3d_msgs::action::NavigateThroughPoses>
{
public:
  using ActionT = nav3d_msgs::action::NavigateThroughPoses;

  NavigateThroughPosesNavigator()
  : BehaviorTreeNavigator() {}

  bool configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr node,
    std::shared_ptr<nav3d_util::OdomSmoother> odom_smoother) override;

  std::string getName() override {return std::string("navigate_through_poses");}

  std::string getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr node) override;

protected:
  bool goalReceived(ActionT::Goal::ConstSharedPtr goal) override;

  void onLoop() override;

  void onPreempt(ActionT::Goal::ConstSharedPtr goal) override;

  void goalCompleted(
    typename ActionT::Result::SharedPtr result,
    const nav3d_behavior_tree::BtStatus final_bt_status) override;

  bool initializeGoalPoses(ActionT::Goal::ConstSharedPtr goal);

  rclcpp::Time start_time_;
  std::string goals_blackboard_id_;
  std::string path_blackboard_id_;
  std::string waypoint_statuses_blackboard_id_;

  std::shared_ptr<nav3d_util::OdomSmoother> odom_smoother_;
};

}  // namespace nav3d_bt_navigator