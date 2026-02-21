#pragma once

#include <string>

#include "nav3d_behavior_tree/bt_action_node.hpp"
#include "nav3d_msgs/action/wait.hpp"

namespace nav3d_behavior_tree
{

class WaitAction : public BtActionNode<nav3d_msgs::action::Wait>
{
  using Action = nav3d_msgs::action::Wait;
  using ActionResult = Action::Result;

public:
  WaitAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  void on_timeout() override;

  void initialize();

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<double>("wait_duration", 1.0, "Wait time"),
        BT::OutputPort<ActionResult::_error_code_type>(
          "error_code_id", "The wait behavior error code"),
        BT::OutputPort<std::string>(
          "error_msg", "The wait behavior error msg"),
      });
  }

  BT::NodeStatus on_success() override;
  BT::NodeStatus on_aborted() override;
  BT::NodeStatus on_cancelled() override;
};

}  // namespace nav3d_behavior_tree