#include <memory>
#include <string>

#include "nav3d_behavior_tree/plugins/action/get_next_few_goals_action.hpp"

namespace nav3d_behavior_tree
{

GetNextFewGoals::GetNextFewGoals(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
}

BT::NodeStatus GetNextFewGoals::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  nav_msgs::msg::Goals input_goals;
  getInput("input_goals", input_goals);

  int num_goals = 0;
  getInput("num_goals", num_goals);

  if (input_goals.goals.empty()) {
    return BT::NodeStatus::FAILURE;
  }

  if (num_goals < 0) {
    num_goals = 0;
  }

  nav_msgs::msg::Goals output_goals;
  output_goals.header = input_goals.header;

  const auto count = static_cast<size_t>(num_goals);
  const auto limit = std::min(count, input_goals.goals.size());
  output_goals.goals.reserve(limit);

  for (size_t i = 0; i < limit; ++i) {
    output_goals.goals.push_back(input_goals.goals[i]);
  }

  setOutput("output_goals", output_goals);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav3d_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav3d_behavior_tree::GetNextFewGoals>("GetNextFewGoals");
}