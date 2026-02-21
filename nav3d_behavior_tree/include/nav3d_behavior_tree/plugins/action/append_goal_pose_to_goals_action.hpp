#pragma once

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav3d_util/geometry_utils.hpp"
#include "nav3d_util/robot_utils.hpp"
#include "nav_msgs/msg/goals.hpp"

namespace nav3d_behavior_tree
{

class AppendGoalPoseToGoals : public BT::ActionNodeBase
{
public:
  AppendGoalPoseToGoals(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal_pose", "Goal pose to append"),
      BT::InputPort<nav_msgs::msg::Goals>("input_goals", "Input goals to append to"),
      BT::OutputPort<nav_msgs::msg::Goals>("output_goals", "Output goals after appending")
    };
  }

private:
  void halt() override {}
  BT::NodeStatus tick() override;
};

}  // namespace nav3d_behavior_tree