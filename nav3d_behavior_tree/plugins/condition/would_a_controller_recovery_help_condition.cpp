#include <memory>

#include "nav3d_behavior_tree/plugins/condition/would_a_controller_recovery_help_condition.hpp"

namespace nav3d_behavior_tree
{

WouldAControllerRecoveryHelp::WouldAControllerRecoveryHelp(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: AreErrorCodesPresent(condition_name, conf)
{
  error_codes_to_check_ = {
    ActionResult::UNKNOWN,
    ActionResult::PATIENCE_EXCEEDED,
    ActionResult::FAILED_TO_MAKE_PROGRESS,
    ActionResult::NO_VALID_CONTROL
  };
}

}  // namespace nav3d_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav3d_behavior_tree::WouldAControllerRecoveryHelp>(
    "WouldAControllerRecoveryHelp");
}