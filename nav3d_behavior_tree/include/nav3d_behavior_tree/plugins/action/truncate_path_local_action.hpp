#pragma once

#include <limits>
#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/json_export.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav3d_behavior_tree/bt_utils.hpp"
#include "nav3d_behavior_tree/json_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"

namespace nav3d_behavior_tree
{

class TruncatePathLocal : public BT::ActionNodeBase
{
public:
  TruncatePathLocal(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    BT::RegisterJsonDefinition<nav_msgs::msg::Path>();
    BT::RegisterJsonDefinition<geometry_msgs::msg::PoseStamped>();

    return {
      BT::InputPort<nav_msgs::msg::Path>("input_path", "Original Path"),
      BT::OutputPort<nav_msgs::msg::Path>(
        "output_path", "Path truncated to a certain distance around robot"),
      BT::InputPort<double>(
        "distance_forward", 8.0,
        "Distance in forward direction"),
      BT::InputPort<double>(
        "distance_backward", 4.0,
        "Distance in backward direction"),
      BT::InputPort<std::string>(
        "robot_frame", "base_link",
        "Robot base frame id"),
      BT::InputPort<double>(
        "transform_tolerance", 0.2,
        "Transform lookup tolerance"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "pose", "Manually specified pose to be used"
        "if overriding current robot pose"),
      BT::InputPort<double>(
        "angular_distance_weight", 0.0,
        "Weight of angular distance relative to positional distance when finding which path "
        "pose is closest to robot. Not applicable on paths without orientations assigned"),
      BT::InputPort<double>(
        "max_robot_pose_search_dist", std::numeric_limits<double>::infinity(),
        "Maximum forward integrated distance along the path (starting from the last detected pose) "
        "to bound the search for the closest pose to the robot. When set to infinity (default), "
        "whole path is searched every time"),
    };
  }

private:
  void halt() override {}
  BT::NodeStatus tick() override;

  bool getRobotPose(std::string path_frame_id, geometry_msgs::msg::PoseStamped & pose);

  static double poseDistance(
    const geometry_msgs::msg::PoseStamped & pose1,
    const geometry_msgs::msg::PoseStamped & pose2,
    const double angular_distance_weight);

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  nav_msgs::msg::Path path_;
  nav_msgs::msg::Path::_poses_type::iterator closest_pose_detection_begin_;
};

}  // namespace nav3d_behavior_tree