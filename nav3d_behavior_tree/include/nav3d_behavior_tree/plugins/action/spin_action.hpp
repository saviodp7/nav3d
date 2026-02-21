#pragma once

#include <string>

#include "nav3d_behavior_tree/bt_action_node.hpp"
#include "nav3d_msgs/action/spin.hpp"

namespace nav3d_behavior_tree
{

class SpinAction : public BtActionNode<nav3d_msgs::action::Spin>
{
  using Action = nav3d_msgs::action::Spin;
  using ActionResult = Action::Result;

public:
  SpinAction(
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
        BT::InputPort<double>("spin_dist", 1.57, "Spin distance"),
        BT::InputPort<double>("time_allowance", 10.0, "Allowed time for spinning"),
        BT::InputPort<bool>("is_recovery", true, "True if recovery"),
        BT::InputPort<bool>("disable_collision_checks", false, "Disable collision checking"),
        BT::OutputPort<ActionResult::_error_code_type>(
          "error_code_id", "The spin behavior error code"),
        BT::OutputPort<std::string>(
          "error_msg", "The spin behavior error msg"),
      });
  }

  BT::NodeStatus on_success() override;
  BT::NodeStatus on_aborted() override;
  BT::NodeStatus on_cancelled() override;

private:
  bool is_recovery_;
};

}  // namespace nav3d_behavior_tree