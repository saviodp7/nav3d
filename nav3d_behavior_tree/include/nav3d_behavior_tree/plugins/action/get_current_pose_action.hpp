#pragma once

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav3d_behavior_tree/bt_utils.hpp"
#include "nav3d_util/geometry_utils.hpp"
#include "nav3d_util/robot_utils.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav3d_behavior_tree
{

class GetCurrentPoseAction : public BT::ActionNodeBase
{
public:
  GetCurrentPoseAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("global_frame", "Global reference frame"),
      BT::InputPort<std::string>("robot_base_frame", "robot base frame"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("current_pose", "Current pose output"),
    };
  }

private:
  void halt() override {}
  BT::NodeStatus tick() override;

  std::string global_frame_;
  std::string robot_base_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  double transform_tolerance_;
};

}  // namespace nav3d_behavior_tree