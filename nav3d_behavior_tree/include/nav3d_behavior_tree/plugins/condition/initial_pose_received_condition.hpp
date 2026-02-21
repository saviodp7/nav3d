#pragma once

#include <string>

#include "behaviortree_cpp/behavior_tree.h"
#include "nav3d_behavior_tree/bt_utils.hpp"

namespace nav3d_behavior_tree
{

class InitialPoseReceived : public BT::ConditionNode
{
public:
  InitialPoseReceived(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<bool>("initial_pose_received")};
  }

  BT::NodeStatus tick() override;
};

}  // namespace nav3d_behavior_tree