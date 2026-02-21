#include <string>

#include "nav3d_behavior_tree/plugins/control/recovery_node.hpp"

namespace nav3d_behavior_tree
{

RecoveryNode::RecoveryNode(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ControlNode::ControlNode(name, conf),
  current_child_idx_(0),
  number_of_retries_(1),
  retry_count_(0)
{
}

BT::NodeStatus RecoveryNode::tick()
{
  getInput("number_of_retries", number_of_retries_);
  const unsigned children_count = children_nodes_.size();

  if (children_count != 2) {
    throw BT::BehaviorTreeException("Recovery Node '" + name() + "' must only have 2 children.");
  }

  setStatus(BT::NodeStatus::RUNNING);

  while (current_child_idx_ < children_count && retry_count_ <= number_of_retries_) {
    TreeNode * child_node = children_nodes_[current_child_idx_];
    const BT::NodeStatus child_status = child_node->executeTick();

    if (current_child_idx_ == 0) {
      switch (child_status) {
        case BT::NodeStatus::SKIPPED:
          halt();
          return BT::NodeStatus::SKIPPED;

        case BT::NodeStatus::SUCCESS:
          ControlNode::haltChild(1);
          halt();
          return BT::NodeStatus::SUCCESS;

        case BT::NodeStatus::RUNNING:
          return BT::NodeStatus::RUNNING;

        case BT::NodeStatus::FAILURE:
          if (retry_count_ < number_of_retries_) {
            ControlNode::haltChild(0);
            current_child_idx_++;
            break;
          } else {
            halt();
            return BT::NodeStatus::FAILURE;
          }

        default:
          throw BT::LogicError("A child node must never return IDLE");
      }

    } else if (current_child_idx_ == 1) {
      switch (child_status) {
        case BT::NodeStatus::SKIPPED:
          current_child_idx_ = 0;
          ControlNode::haltChild(1);
          return BT::NodeStatus::FAILURE;

        case BT::NodeStatus::RUNNING:
          return child_status;

        case BT::NodeStatus::SUCCESS:
          ControlNode::haltChild(1);
          retry_count_++;
          current_child_idx_ = 0;
          break;

        case BT::NodeStatus::FAILURE:
          halt();
          return BT::NodeStatus::FAILURE;

        default:
          throw BT::LogicError("A child node must never return IDLE");
      }
    }
  }

  halt();
  return BT::NodeStatus::FAILURE;
}

void RecoveryNode::halt()
{
  ControlNode::halt();
  retry_count_ = 0;
  current_child_idx_ = 0;
}

}  // namespace nav3d_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav3d_behavior_tree::RecoveryNode>("RecoveryNode");
}