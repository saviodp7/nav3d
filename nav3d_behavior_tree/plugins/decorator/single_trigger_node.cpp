#include <chrono>
#include <string>

#include "nav3d_behavior_tree/plugins/decorator/single_trigger_node.hpp"

namespace nav3d_behavior_tree
{

SingleTrigger::SingleTrigger(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf),
  first_time_(true)
{
}

BT::NodeStatus SingleTrigger::tick()
{
  if (!BT::isStatusActive(status())) {
    first_time_ = true;
  }

  setStatus(BT::NodeStatus::RUNNING);

  if (first_time_) {
    const BT::NodeStatus child_state = child_node_->executeTick();

    switch (child_state) {
      case BT::NodeStatus::SKIPPED:
      case BT::NodeStatus::RUNNING:
        return child_state;

      case BT::NodeStatus::FAILURE:
      case BT::NodeStatus::SUCCESS:
        first_time_ = false;
        return child_state;

      default:
        first_time_ = false;
        return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::FAILURE;
}

}

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav3d_behavior_tree::SingleTrigger>("SingleTrigger");
}