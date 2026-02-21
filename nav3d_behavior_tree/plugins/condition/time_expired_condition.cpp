#include <string>
#include <memory>

#include "behaviortree_cpp/condition_node.h"

#include "nav3d_behavior_tree/plugins/condition/time_expired_condition.hpp"

namespace nav3d_behavior_tree
{

TimeExpiredCondition::TimeExpiredCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  period_(1.0)
{
}

void TimeExpiredCondition::initialize()
{
  getInput("seconds", period_);
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  start_ = node_->now();
}

BT::NodeStatus TimeExpiredCondition::tick()
{
  const bool was_active = BT::isStatusActive(status());
  if (!was_active) {
    initialize();
    start_ = node_->now();
    return BT::NodeStatus::FAILURE;
  }

  const double seconds = (node_->now() - start_).seconds();

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
  factory.registerNodeType<nav3d_behavior_tree::TimeExpiredCondition>("TimeExpired");
}