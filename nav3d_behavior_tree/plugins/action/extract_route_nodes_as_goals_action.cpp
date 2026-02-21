#include <memory>
#include <string>

#include "nav3d_behavior_tree/plugins/action/extract_route_nodes_as_goals_action.hpp"

namespace nav3d_behavior_tree
{

ExtractRouteNodesAsGoals::ExtractRouteNodesAsGoals(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
}

BT::NodeStatus ExtractRouteNodesAsGoals::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  nav3d_msgs::msg::Route route;
  getInput("route", route);

  if (route.nodes.empty()) {
    return BT::NodeStatus::FAILURE;
  }

  nav_msgs::msg::Goals goals;
  goals.header = route.header;
  goals.goals.reserve(route.nodes.size());

  for (const auto & node : route.nodes) {
    geometry_msgs::msg::PoseStamped goal;
    goal.header = route.header;
    goal.pose.position.x = node.position.x;
    goal.pose.position.y = node.position.y;
    goals.goals.push_back(goal);
  }

  setOutput("goals", goals);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav3d_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav3d_behavior_tree::ExtractRouteNodesAsGoals>(
    "ExtractRouteNodesAsGoals");
}