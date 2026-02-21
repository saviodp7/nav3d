#pragma once

#include <string>

#include "nav3d_behavior_tree/bt_action_node.hpp"
#include "nav3d_msgs/action/navigate_to_pose.hpp"

namespace nav3d_behavior_tree
{

class NavigateToPoseAction : public BtActionNode<nav3d_msgs::action::NavigateToPose>
{
  using Action = nav3d_msgs::action::NavigateToPose;
  using ActionResult = Action::Result;

public:
  NavigateToPoseAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  BT::NodeStatus on_success() override;
  BT::NodeStatus on_aborted() override;
  BT::NodeStatus on_cancelled() override;
  void on_timeout() override;

  static BT::PortsList providedPorts()
  {
    BT::RegisterJsonDefinition<geometry_msgs::msg::PoseStamped>();

    return providedBasicPorts(
      {
        BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination to plan to"),
        BT::InputPort<std::string>("behavior_tree", "Behavior tree to run"),
        BT::OutputPort<ActionResult::_error_code_type>(
          "error_code_id", "Navigate to pose error code"),
        BT::OutputPort<std::string>(
          "error_msg", "Navigate to pose error msg"),
      });
  }
};

}  // namespace nav3d_behavior_tree