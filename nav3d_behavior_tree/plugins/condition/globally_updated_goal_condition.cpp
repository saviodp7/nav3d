#include <string>
#include <vector>

#include "nav3d_behavior_tree/plugins/condition/globally_updated_goal_condition.hpp"

namespace nav3d_behavior_tree
{

GloballyUpdatedGoalCondition::GloballyUpdatedGoalCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  first_time(true)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::NodeStatus GloballyUpdatedGoalCondition::tick()
{
  if (first_time) {
    first_time = false;
    BT::getInputOrBlackboard("goals", goals_);
    BT::getInputOrBlackboard("goal", goal_);
    return BT::NodeStatus::SUCCESS;
  }

  nav_msgs::msg::Goals current_goals;
  BT::getInputOrBlackboard("goals", current_goals);
  geometry_msgs::msg::PoseStamped current_goal;
  BT::getInputOrBlackboard("goal", current_goal);

  if (goal_ != current_goal || goals_ != current_goals) {
    goal_ = current_goal;
    goals_ = current_goals;
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace nav3d_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav3d_behavior_tree::GloballyUpdatedGoalCondition>("GlobalUpdatedGoal");
}