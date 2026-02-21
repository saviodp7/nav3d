#pragma once

#include <string>

#include "nav3d_behavior_tree/bt_action_node.hpp"
#include "nav3d_msgs/action/compute_path_to_pose.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav3d_behavior_tree
{

class ComputePathToPoseAction : public BtActionNode<nav3d_msgs::action::ComputePathToPose>
{
  using Action = nav3d_msgs::action::ComputePathToPose;
  using ActionResult = Action::Result;

public:
  ComputePathToPoseAction(
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
        BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination to plan to"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>(
          "start",
          "Used as the planner start pose instead of the current robot pose, if use_start is"
          " not false (i.e. not provided or set to true)"),
        BT::InputPort<bool>(
          "use_start", "For using or not using (i.e. ignoring) the provided start pose"),
        BT::InputPort<std::string>(
          "planner_id", "",
          "Mapped name to the planner plugin type to use"),
        BT::OutputPort<nav_msgs::msg::Path>("path", "Path created by ComputePathToPose node"),
        BT::OutputPort<ActionResult::_error_code_type>(
          "error_code_id", "The compute path to pose error code"),
        BT::OutputPort<std::string>(
          "error_msg", "The compute path to pose error msg"),
      });
  }
};

}  // namespace nav3d_behavior_tree