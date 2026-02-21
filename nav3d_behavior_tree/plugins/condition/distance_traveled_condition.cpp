#include <memory>
#include <string>

#include "nav3d_util/geometry_utils.hpp"
#include "nav3d_util/robot_utils.hpp"

#include "nav3d_behavior_tree/plugins/condition/distance_traveled_condition.hpp"

namespace nav3d_behavior_tree
{

DistanceTraveledCondition::DistanceTraveledCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  distance_(1.0),
  transform_tolerance_(0.1)
{
}

void DistanceTraveledCondition::initialize()
{
  getInput("distance", distance_);

  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  node_->get_parameter("transform_tolerance", transform_tolerance_);

  global_frame_ = BT::deconflictPortAndParamFrame<std::string>(node_, "global_frame", this);
  robot_base_frame_ = BT::deconflictPortAndParamFrame<std::string>(node_, "robot_base_frame", this);
}

BT::NodeStatus DistanceTraveledCondition::tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
    if (!nav3d_util::getCurrentPose(
          start_pose_, *tf_, global_frame_, robot_base_frame_, transform_tolerance_))
    {
      RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
    }
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav3d_util::getCurrentPose(
        current_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
    return BT::NodeStatus::FAILURE;
  }

  const double travelled =
    nav3d_util::geometry_utils::euclidean_distance(start_pose_.pose, current_pose.pose);

  if (travelled < distance_) {
    return BT::NodeStatus::FAILURE;
  }

  start_pose_ = current_pose;
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav3d_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav3d_behavior_tree::DistanceTraveledCondition>("DistanceTraveled");
}