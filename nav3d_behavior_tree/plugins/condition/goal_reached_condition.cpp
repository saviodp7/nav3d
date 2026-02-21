#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav3d_util/node_utils.hpp"
#include "nav3d_util/robot_utils.hpp"

#include "nav3d_behavior_tree/plugins/condition/goal_reached_condition.hpp"

namespace nav3d_behavior_tree
{

GoalReachedCondition::GoalReachedCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  robot_base_frame_ = BT::deconflictPortAndParamFrame<std::string>(node, "robot_base_frame", this);
}

GoalReachedCondition::~GoalReachedCondition()
{
  cleanup();
}

void GoalReachedCondition::initialize()
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  nav3d_util::declare_parameter_if_not_declared(
    node_, "goal_reached_tol", rclcpp::ParameterValue(0.25));
  node_->get_parameter_or<double>("goal_reached_tol", goal_reached_tol_, 0.25);

  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  node_->get_parameter("transform_tolerance", transform_tolerance_);
}

BT::NodeStatus GoalReachedCondition::tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
  }

  return isGoalReached() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

bool GoalReachedCondition::isGoalReached()
{
  geometry_msgs::msg::PoseStamped goal;
  getInput("goal", goal);

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav3d_util::getCurrentPose(
        current_pose, *tf_, goal.header.frame_id, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
    return false;
  }

  const double dx = goal.pose.position.x - current_pose.pose.position.x;
  const double dy = goal.pose.position.y - current_pose.pose.position.y;
  return (dx * dx + dy * dy) <= (goal_reached_tol_ * goal_reached_tol_);
}

}  // namespace nav3d_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav3d_behavior_tree::GoalReachedCondition>("GoalReached");
}