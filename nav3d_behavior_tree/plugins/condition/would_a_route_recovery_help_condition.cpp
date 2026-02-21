#include <memory>

#include "nav3d_behavior_tree/plugins/condition/would_a_route_recovery_help_condition.hpp"

namespace nav3d_behavior_tree
{

WouldARouteRecoveryHelp::WouldARouteRecoveryHelp(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: AreErrorCodesPresent(condition_name, conf)
{
  error_codes_to_check_ = {
    ActionResult::UNKNOWN,
    ActionResult::NO_VALID_ROUTE,
    ActionResult::TIMEOUT,
    TrackActionResult::UNKNOWN,
    TrackActionResult::TIMEOUT,
    TrackActionResult::NO_VALID_ROUTE
  };
}

}  // namespace nav3d_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav3d_behavior_tree::WouldARouteRecoveryHelp>(
    "WouldARouteRecoveryHelp");
}