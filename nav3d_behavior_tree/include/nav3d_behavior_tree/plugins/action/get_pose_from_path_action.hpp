#pragma once

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/json_export.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav3d_behavior_tree/bt_utils.hpp"
#include "nav3d_behavior_tree/json_utils.hpp"
#include "nav3d_util/geometry_utils.hpp"
#include "nav3d_util/robot_utils.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav3d_behavior_tree
{

class GetPoseFromPath : public BT::ActionNodeBase
{
public:
  GetPoseFromPath(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    BT::RegisterJsonDefinition<geometry_msgs::msg::PoseStamped>();
    BT::RegisterJsonDefinition<nav_msgs::msg::Path>();

    return {
      BT::InputPort<nav_msgs::msg::Path>("path", "Path to extract pose from"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose", "Stamped Extracted Pose"),
      BT::InputPort<int>("index", 0, "Index of pose to extract from. -1 is end of list"),
    };
  }

private:
  void halt() override {}
  BT::NodeStatus tick() override;
};

}  // namespace nav3d_behavior_tree