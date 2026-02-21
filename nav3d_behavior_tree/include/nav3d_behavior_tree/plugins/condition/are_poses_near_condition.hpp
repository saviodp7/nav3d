#pragma once

#include <memory>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "nav3d_behavior_tree/bt_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace nav3d_behavior_tree
{

class ArePosesNearCondition : public BT::ConditionNode
{
public:
  ArePosesNearCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  ~ArePosesNearCondition() override = default;

  BT::NodeStatus tick() override;

  void initialize();

  bool arePosesNearby();

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("ref_pose", "Destination"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("target_pose", "Destination"),
      BT::InputPort<std::string>("global_frame", "Global frame"),
      BT::InputPort<double>("tolerance", 0.5, "Tolerance")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  double transform_tolerance_;
  std::string global_frame_;
};

}  // namespace nav3d_behavior_tree