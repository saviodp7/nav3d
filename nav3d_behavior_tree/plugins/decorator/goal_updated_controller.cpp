#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp/decorator_node.h"
#include "nav3d_behavior_tree/plugins/decorator/goal_updated_controller.hpp"

namespace nav3d_behavior_tree
{

GoalUpdatedController::GoalUpdatedController(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf)
{
}

BT::NodeStatus GoalUpdatedController::tick()
{
  if (!BT::isStatusActive(status())) {
    BT::getInputOrBlackboard("goals", goals_);
    BT::getInputOrBlackboard("goal", goal_);
    goal_was_updated_ = true;
  }

  setStatus(BT::NodeStatus::RUNNING);

  nav_msgs::msg::Goals current_goals;
  BT::getInputOrBlackboard("goals", current_goals);
  geometry_msgs::msg::PoseStamped current_goal;
  BT::getInputOrBlackboard("goal", current_goal);

  if (goal_ != current_goal || goals_ != current_goals) {
    goal_ = current_goal;
    goals_ = current_goals;
    goal_was_updated_ = true;
  }

  if ((child_node_->status() == BT::NodeStatus::RUNNING) || goal_was_updated_) {
    goal_was_updated_ = false;
    return child_node_->executeTick();
  }

  return status();
}

}

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav3d_behavior_tree::GoalUpdatedController>("GoalUpdatedController");
}