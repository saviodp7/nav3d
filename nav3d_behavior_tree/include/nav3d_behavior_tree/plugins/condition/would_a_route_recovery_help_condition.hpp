#pragma once

#include <string>

#include "nav3d_behavior_tree/plugins/condition/are_error_codes_present_condition.hpp"
#include "nav3d_msgs/action/compute_and_track_route.hpp"
#include "nav3d_msgs/action/compute_route.hpp"

namespace nav3d_behavior_tree
{

class WouldARouteRecoveryHelp : public AreErrorCodesPresent
{
  using Action = nav3d_msgs::action::ComputeRoute;
  using ActionResult = Action::Result;
  using TrackAction = nav3d_msgs::action::ComputeAndTrackRoute;
  using TrackActionResult = TrackAction::Result;

public:
  WouldARouteRecoveryHelp(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  WouldARouteRecoveryHelp() = delete;
};

}  // namespace nav3d_behavior_tree