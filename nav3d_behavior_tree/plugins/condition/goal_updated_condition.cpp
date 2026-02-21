#include <string>
#include <vector>

#include "nav3d_behavior_tree/plugins/condition/goal_updated_condition.hpp"

namespace nav3d_behavior_tree
{

GoalUpdatedCondition::GoalUpdatedCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{}

BT::NodeStatus GoalUpdatedCondition::tick()
{
  const bool was_active = BT::isStatusActive(status());
  if (!was_active) {
    BT::getInputOrBlackboard("goals", goals_);
    BT::getInputOrBlackboard("goal", goal_);
    return BT::NodeStatus::FAILURE;
  }

  nav_msgs::msg::Goals current_goals;
  geometry_msgs::msg::PoseStamped current_goal;
  BT::getInputOrBlackboard("goals", current_goals);
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
  factory.registerNodeType<nav3d_behavior_tree::GoalUpdatedCondition>("GoalUpdated");
}