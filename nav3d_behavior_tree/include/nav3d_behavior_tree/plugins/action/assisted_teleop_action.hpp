#pragma once

#include <string>

#include "nav3d_behavior_tree/bt_action_node.hpp"
#include "nav3d_msgs/action/assisted_teleop.hpp"

namespace nav3d_behavior_tree
{

class AssistedTeleopAction : public BtActionNode<nav3d_msgs::action::AssistedTeleop>
{
  using Action = nav3d_msgs::action::AssistedTeleop;
  using ActionResult = Action::Result;

public:
  AssistedTeleopAction(
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
        BT::InputPort<double>("time_allowance", 10.0, "Allowed time for running assisted teleop"),
        BT::InputPort<bool>("is_recovery", false, "If true the recovery count will be incremented"),
        BT::OutputPort<ActionResult::_error_code_type>(
          "error_code_id", "The assisted teleop behavior server error code"),
        BT::OutputPort<std::string>(
          "error_msg", "The assisted teleop behavior server error msg"),
      });
  }

private:
  bool is_recovery_;
};

}  // namespace nav3d_behavior_tree