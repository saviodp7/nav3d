#pragma once

#include <string>
#include <vector>

#include "nav3d_behavior_tree/bt_action_node.hpp"
#include "nav3d_msgs/action/compute_path_through_poses.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav3d_behavior_tree
{

class ComputePathThroughPosesAction
  : public BtActionNode<nav3d_msgs::action::ComputePathThroughPoses>
{
  using Action = nav3d_msgs::action::ComputePathThroughPoses;
  using ActionResult = Action::Result;

public:
  ComputePathThroughPosesAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  BT::NodeStatus on_success() override;
  BT::NodeStatus on_aborted() override;
  BT::NodeStatus on_cancelled() override;
  void on_timeout() override;

  void halt() override;

  static BT::PortsList providedPorts()
  {
    BT::RegisterJsonDefinition<nav_msgs::msg::Path>();
    BT::RegisterJsonDefinition<geometry_msgs::msg::PoseStamped>();

    return providedBasicPorts(
      {
        BT::InputPort<nav_msgs::msg::Goals>(
          "goals",
          "Destinations to plan through"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>(
          "start", "Start pose of the path if overriding current robot pose"),
        BT::InputPort<std::string>(
          "planner_id", "",
          "Mapped name to the planner plugin type to use"),
        BT::OutputPort<nav_msgs::msg::Path>("path", "Path created by ComputePathThroughPoses node"),
        BT::OutputPort<ActionResult::_error_code_type>(
          "error_code_id", "The compute path through poses error code"),
        BT::OutputPort<std::string>(
          "error_msg", "The compute path through poses error msg"),
      });
  }
};

}  // namespace nav3d_behavior_tree