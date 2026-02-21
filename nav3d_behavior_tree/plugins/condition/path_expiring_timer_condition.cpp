#include <string>
#include <memory>

#include "behaviortree_cpp/condition_node.h"

#include "nav3d_behavior_tree/plugins/condition/path_expiring_timer_condition.hpp"

namespace nav3d_behavior_tree
{

PathExpiringTimerCondition::PathExpiringTimerCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  period_(1.0),
  first_time_(true)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::NodeStatus PathExpiringTimerCondition::tick()
{
  if (first_time_) {
    getInput("seconds", period_);
    getInput("path", prev_path_);
    first_time_ = false;
    start_ = node_->now();
    return BT::NodeStatus::FAILURE;
  }

  nav_msgs::msg::Path path;
  getInput("path", path);

  if (prev_path_ != path) {
    prev_path_ = path;
    start_ = node_->now();
  }

  auto elapsed = node_->now() - start_;
  const double seconds = elapsed.seconds();

  if (seconds < period_) {
    return BT::NodeStatus::FAILURE;
  }

  start_ = node_->now();
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav3d_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav3d_behavior_tree::PathExpiringTimerCondition>("PathExpiringTimer");
}