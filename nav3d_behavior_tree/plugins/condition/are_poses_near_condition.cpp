#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav3d_util/robot_utils.hpp"

#include "nav3d_behavior_tree/plugins/condition/are_poses_near_condition.hpp"

namespace nav3d_behavior_tree
{

ArePosesNearCondition::ArePosesNearCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  global_frame_ = BT::deconflictPortAndParamFrame<std::string>(node, "global_frame", this);
}

void ArePosesNearCondition::initialize()
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  node_->get_parameter("transform_tolerance", transform_tolerance_);
}

BT::NodeStatus ArePosesNearCondition::tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
  }

  return arePosesNearby() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

bool ArePosesNearCondition::arePosesNearby()
{
  geometry_msgs::msg::PoseStamped pose1, pose2;
  double tol = 0.0;

  getInput("ref_pose", pose1);
  getInput("target_pose", pose2);
  getInput("tolerance", tol);

  if (pose1.header.frame_id != pose2.header.frame_id) {
    if (!nav3d_util::transformPoseInTargetFrame(
          pose1, pose1, *tf_, global_frame_, transform_tolerance_) ||
        !nav3d_util::transformPoseInTargetFrame(
          pose2, pose2, *tf_, global_frame_, transform_tolerance_))
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to transform poses to the same frame");
      return false;
    }
  }

  const double dx = pose1.pose.position.x - pose2.pose.position.x;
  const double dy = pose1.pose.position.y - pose2.pose.position.y;
  return (dx * dx + dy * dy) <= (tol * tol);
}

}  // namespace nav3d_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav3d_behavior_tree::ArePosesNearCondition>("ArePosesNear");
}