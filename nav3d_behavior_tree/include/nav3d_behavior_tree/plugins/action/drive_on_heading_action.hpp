#pragma once

#include <string>

#include "nav3d_behavior_tree/bt_action_node.hpp"
#include "nav3d_msgs/action/drive_on_heading.hpp"

namespace nav3d_behavior_tree
{

class DriveOnHeadingAction : public BtActionNode<nav3d_msgs::action::DriveOnHeading>
{
  using Action = nav3d_msgs::action::DriveOnHeading;
  using ActionResult = Action::Result;

public:
  DriveOnHeadingAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void initialize();

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<double>("dist_to_travel", 0.15, "Distance to travel"),
        BT::InputPort<double>("speed", 0.025, "Speed at which to travel"),
        BT::InputPort<double>("time_allowance", 10.0, "Allowed time for driving on heading"),
        BT::InputPort<bool>("disable_collision_checks", false, "Disable collision checking"),
        BT::OutputPort<Action::Result::_error_code_type>(
          "error_code_id", "The drive on heading behavior server error code"),
        BT::OutputPort<std::string>(
          "error_msg", "The drive on heading behavior server error msg"),
      });
  }

  void on_tick() override;
  BT::NodeStatus on_success() override;
  BT::NodeStatus on_aborted() override;
  BT::NodeStatus on_cancelled() override;
  void on_timeout() override;
};

}  // namespace nav3d_behavior_tree