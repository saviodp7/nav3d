#include <chrono>
#include <string>
#include <memory>
#include <cmath>

#include "nav3d_util/robot_utils.hpp"
#include "nav3d_util/geometry_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"

#include "behaviortree_cpp/decorator_node.h"

#include "nav3d_behavior_tree/plugins/decorator/distance_controller.hpp"

namespace nav3d_behavior_tree
{

DistanceController::DistanceController(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf),
  distance_(1.0),
  first_time_(false)
{
  getInput("distance", distance_);
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  node_->get_parameter("transform_tolerance", transform_tolerance_);

  global_frame_ = BT::deconflictPortAndParamFrame<std::string>(
    node_, "global_frame", this);
  robot_base_frame_ = BT::deconflictPortAndParamFrame<std::string>(
    node_, "robot_base_frame", this);
}

inline BT::NodeStatus DistanceController::tick()
{
  if (!BT::isStatusActive(status())) {
    if (!nav3d_util::getCurrentPose(
        start_pose_, *tf_, global_frame_, robot_base_frame_,
        transform_tolerance_))
    {
      RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
      return BT::NodeStatus::FAILURE;
    }
    first_time_ = true;
  }

  setStatus(BT::NodeStatus::RUNNING);

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav3d_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
    return BT::NodeStatus::FAILURE;
  }

  auto travelled = nav3d_util::geometry_utils::euclidean_distance(
    start_pose_.pose, current_pose.pose);

  if (first_time_ || (child_node_->status() == BT::NodeStatus::RUNNING) ||
    travelled >= distance_)
  {
    first_time_ = false;
    const BT::NodeStatus child_state = child_node_->executeTick();

    switch (child_state) {
      case BT::NodeStatus::SKIPPED:
      case BT::NodeStatus::RUNNING:
        return child_state;

      case BT::NodeStatus::SUCCESS:
        if (!nav3d_util::getCurrentPose(
            start_pose_, *tf_, global_frame_, robot_base_frame_,
            transform_tolerance_))
        {
          RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
          return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::SUCCESS;

      case BT::NodeStatus::FAILURE:
      default:
        return BT::NodeStatus::FAILURE;
    }
  }

  return status();
}

}

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav3d_behavior_tree::DistanceController>("DistanceController");
}