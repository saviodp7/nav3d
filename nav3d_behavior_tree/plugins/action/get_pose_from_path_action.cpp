#include <memory>
#include <string>

#include "nav3d_behavior_tree/plugins/action/get_pose_from_path_action.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav3d_behavior_tree
{

GetPoseFromPath::GetPoseFromPath(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
}

BT::NodeStatus GetPoseFromPath::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  nav_msgs::msg::Path input_path;
  getInput("path", input_path);

  int pose_index = 0;
  getInput("index", pose_index);

  if (input_path.poses.empty()) {
    return BT::NodeStatus::FAILURE;
  }

  if (pose_index < 0) {
    pose_index = static_cast<int>(input_path.poses.size()) + pose_index;
  }

  if (pose_index < 0 || static_cast<size_t>(pose_index) >= input_path.poses.size()) {
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PoseStamped output_pose = input_path.poses[static_cast<size_t>(pose_index)];

  if (output_pose.header.frame_id.empty()) {
    output_pose.header.frame_id = input_path.header.frame_id;
  }

  setOutput("pose", output_pose);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav3d_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav3d_behavior_tree::GetPoseFromPath>("GetPoseFromPath");
}