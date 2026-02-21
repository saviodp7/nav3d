#pragma once

#include <atomic>
#include <memory>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace nav3d_behavior_tree
{

class TransformAvailableCondition : public BT::ConditionNode
{
public:
  TransformAvailableCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  TransformAvailableCondition() = delete;

  ~TransformAvailableCondition() override;

  BT::NodeStatus tick() override;

  void initialize();

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("child", std::string(), "Child frame for transform"),
      BT::InputPort<std::string>("parent", std::string(), "parent frame for transform")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  std::atomic<bool> was_found_;

  std::string child_frame_;
  std::string parent_frame_;
};

}  // namespace nav3d_behavior_tree