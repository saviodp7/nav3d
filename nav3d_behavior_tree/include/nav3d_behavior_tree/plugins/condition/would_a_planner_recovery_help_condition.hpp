#pragma once

#include <string>

#include "nav3d_behavior_tree/plugins/condition/are_error_codes_present_condition.hpp"
#include "nav3d_msgs/action/compute_path_through_poses.hpp"
#include "nav3d_msgs/action/compute_path_to_pose.hpp"

namespace nav3d_behavior_tree
{

class WouldAPlannerRecoveryHelp : public AreErrorCodesPresent
{
  using Action = nav3d_msgs::action::ComputePathToPose;
  using ActionResult = Action::Result;
  using ThroughAction = nav3d_msgs::action::ComputePathThroughPoses;
  using ThroughActionResult = Action::Result;

public:
  WouldAPlannerRecoveryHelp(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  WouldAPlannerRecoveryHelp() = delete;
};

}  // namespace nav3d_behavior_tree