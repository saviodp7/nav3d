#pragma once

#include <string>
#include <vector>

#include "nav3d_behavior_tree/bt_action_node.hpp"
#include "nav3d_msgs/action/navigate_through_poses.hpp"
#include "nav_msgs/msg/goals.hpp"

namespace nav3d_behavior_tree
{

class NavigateThroughPosesAction : public BtActionNode<nav3d_msgs::action::NavigateThroughPoses>
{
  using Action = nav3d_msgs::action::NavigateThroughPoses;
  using ActionResult = Action::Result;

public:
  NavigateThroughPosesAction(
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
    BT::RegisterJsonDefinition<nav_msgs::msg::Goals>();

    return providedBasicPorts(
      {
        BT::InputPort<nav_msgs::msg::Goals>(
          "goals", "Destinations to plan through"),
        BT::InputPort<std::string>("behavior_tree", "Behavior tree to run"),
        BT::OutputPort<ActionResult::_error_code_type>(
          "error_code_id", "The navigate through poses error code"),
        BT::OutputPort<std::string>(
          "error_msg", "The navigate through poses error msg"),
      });
  }
};

}  // namespace nav3d_behavior_tree