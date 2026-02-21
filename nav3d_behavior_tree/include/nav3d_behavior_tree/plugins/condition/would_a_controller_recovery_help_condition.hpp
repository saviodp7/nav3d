#pragma once

#include <string>

#include "nav3d_behavior_tree/plugins/condition/are_error_codes_present_condition.hpp"
#include "nav3d_msgs/action/follow_path.hpp"

namespace nav3d_behavior_tree
{

class WouldAControllerRecoveryHelp : public AreErrorCodesPresent
{
  using Action = nav3d_msgs::action::FollowPath;
  using ActionResult = Action::Result;

public:
  WouldAControllerRecoveryHelp(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  WouldAControllerRecoveryHelp() = delete;
};

}  // namespace nav3d_behavior_tree