#include <memory>

#include "nav3d_behavior_tree/plugins/condition/would_a_smoother_recovery_help_condition.hpp"

namespace nav3d_behavior_tree
{

WouldASmootherRecoveryHelp::WouldASmootherRecoveryHelp(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: AreErrorCodesPresent(condition_name, conf)
{
  error_codes_to_check_ = {
    ActionResult::UNKNOWN,
    ActionResult::TIMEOUT,
    ActionResult::FAILED_TO_SMOOTH_PATH,
    ActionResult::SMOOTHED_PATH_IN_COLLISION
  };
}

}  // namespace nav3d_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav3d_behavior_tree::WouldASmootherRecoveryHelp>(
    "WouldASmootherRecoveryHelp");
}