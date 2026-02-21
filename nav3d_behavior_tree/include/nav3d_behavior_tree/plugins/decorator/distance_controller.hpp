#pragma once

#include <memory>
#include <string>

#include "behaviortree_cpp/decorator_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav3d_behavior_tree/bt_utils.hpp"
#include "tf2_ros/buffer.h"

namespace nav3d_behavior_tree
{

class DistanceController : public BT::DecoratorNode
{
public:
  DistanceController(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("distance", 1.0, "Distance"),
      BT::InputPort<std::string>("global_frame", "Global frame"),
      BT::InputPort<std::string>("robot_base_frame", "Robot base frame")
    };
  }

private:
  BT::NodeStatus tick() override;

  rclcpp::Node::SharedPtr node_;

  std::shared_ptr<tf2_ros::Buffer> tf_;
  double transform_tolerance_;

  geometry_msgs::msg::PoseStamped start_pose_;
  double distance_;
  std::string global_frame_;
  std::string robot_base_frame_;

  bool first_time_;
};

}  // namespace nav3d_behavior_tree