#include <memory>
#include <string>

#include "nav3d_behavior_tree/plugins/action/get_current_pose_action.hpp"
#include "nav3d_util/robot_utils.hpp"

namespace nav3d_behavior_tree
{

GetCurrentPoseAction::GetCurrentPoseAction(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  node->get_parameter("transform_tolerance", transform_tolerance_);
  global_frame_ = BT::deconflictPortAndParamFrame<std::string>(
    node, "global_frame", this);
  robot_base_frame_ = BT::deconflictPortAndParamFrame<std::string>(
    node, "robot_base_frame", this);
}

BT::NodeStatus GetCurrentPoseAction::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav3d_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_WARN(
      config().blackboard->get<rclcpp::Node::SharedPtr>("node")->get_logger(),
      "Current robot pose is not available.");
    return BT::NodeStatus::FAILURE;
  }

  setOutput("current_pose", current_pose);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav3d_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav3d_behavior_tree::GetCurrentPoseAction>("GetCurrentPose");
}