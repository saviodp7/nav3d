#pragma once

#include <string>

#include "nav3d_behavior_tree/plugins/condition/are_error_codes_present_condition.hpp"
#include "nav3d_msgs/action/smooth_path.hpp"

namespace nav3d_behavior_tree
{

class WouldASmootherRecoveryHelp : public AreErrorCodesPresent
{
  using Action = nav3d_msgs::action::SmoothPath;
  using ActionResult = Action::Result;

public:
  WouldASmootherRecoveryHelp(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  WouldASmootherRecoveryHelp() = delete;
};

}  // namespace nav3d_behavior_tree