#pragma once

#include <string>

#include "nav3d_behavior_tree/bt_action_node.hpp"
#include "nav3d_msgs/action/back_up.hpp"

namespace nav3d_behavior_tree
{

class BackUpAction : public BtActionNode<nav3d_msgs::action::BackUp>
{
  using Action = nav3d_msgs::action::BackUp;
  using ActionResult = Action::Result;

public:
  BackUpAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  BT::NodeStatus on_success() override;
  BT::NodeStatus on_aborted() override;
  BT::NodeStatus on_cancelled() override;
  void on_timeout() override;

  void initialize();

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<double>("backup_dist", 0.15, "Distance to backup"),
        BT::InputPort<double>("backup_speed", 0.025, "Speed at which to backup"),
        BT::InputPort<double>("time_allowance", 10.0, "Allowed time for reversing"),
        BT::InputPort<bool>("disable_collision_checks", false, "Disable collision checking"),
        BT::OutputPort<ActionResult::_error_code_type>(
          "error_code_id", "The back up behavior server error code"),
        BT::OutputPort<std::string>(
          "error_msg", "The back up behavior server error msg"),
      });
  }
};

}  // namespace nav3d_behavior_tree