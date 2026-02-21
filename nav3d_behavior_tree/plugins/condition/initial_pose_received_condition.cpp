#include "nav3d_behavior_tree/plugins/condition/initial_pose_received_condition.hpp"

namespace nav3d_behavior_tree
{

InitialPoseReceived::InitialPoseReceived(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::NodeStatus InitialPoseReceived::tick()
{
  bool init_pose_received = false;
  BT::getInputOrBlackboard("initial_pose_received", init_pose_received);
  return init_pose_received ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace nav3d_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav3d_behavior_tree::InitialPoseReceived>("InitialPoseReceived");
}