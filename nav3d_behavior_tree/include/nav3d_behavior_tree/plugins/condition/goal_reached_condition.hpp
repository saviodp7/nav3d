#pragma once

#include <memory>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_cpp/json_export.h"
#include "nav3d_behavior_tree/bt_utils.hpp"
#include "nav3d_behavior_tree/json_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace nav3d_behavior_tree
{

class GoalReachedCondition : public BT::ConditionNode
{
public:
  GoalReachedCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  GoalReachedCondition() = delete;

  ~GoalReachedCondition() override;

  BT::NodeStatus tick() override;

  void initialize();

  bool isGoalReached();

  static BT::PortsList providedPorts()
  {
    BT::RegisterJsonDefinition<geometry_msgs::msg::PoseStamped>();

    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination"),
      BT::InputPort<std::string>("robot_base_frame", "Robot base frame")
    };
  }

protected:
  void cleanup() {}

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  double goal_reached_tol_;
  double transform_tolerance_;
  std::string robot_base_frame_;
};

}  // namespace nav3d_behavior_tree