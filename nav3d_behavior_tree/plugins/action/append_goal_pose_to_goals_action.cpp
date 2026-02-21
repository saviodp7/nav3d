#include <memory>
#include <string>

#include "nav3d_behavior_tree/plugins/action/append_goal_pose_to_goals_action.hpp"

namespace nav3d_behavior_tree
{

AppendGoalPoseToGoals::AppendGoalPoseToGoals(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
}

BT::NodeStatus AppendGoalPoseToGoals::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  geometry_msgs::msg::PoseStamped goal_pose;
  getInput("goal_pose", goal_pose);

  nav_msgs::msg::Goals input;
  getInput("input_goals", input);

  nav_msgs::msg::Goals output = input;
  output.goals.push_back(goal_pose);

  setOutput("output_goals", output);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav3d_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav3d_behavior_tree::AppendGoalPoseToGoals>(
    "AppendGoalPoseToGoals");
}