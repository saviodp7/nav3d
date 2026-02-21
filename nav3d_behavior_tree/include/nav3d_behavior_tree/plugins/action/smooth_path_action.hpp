#pragma once

#include <string>

#include "nav3d_behavior_tree/bt_action_node.hpp"
#include "nav3d_msgs/action/smooth_path.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav3d_behavior_tree
{

class SmoothPathAction : public nav3d_behavior_tree::BtActionNode<nav3d_msgs::action::SmoothPath>
{
  using Action = nav3d_msgs::action::SmoothPath;
  using ActionResult = Action::Result;

public:
  SmoothPathAction(
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
    BT::RegisterJsonDefinition<nav_msgs::msg::Path>();

    return providedBasicPorts(
      {
        BT::InputPort<nav_msgs::msg::Path>("unsmoothed_path", "Path to be smoothed"),
        BT::InputPort<double>("max_smoothing_duration", 3.0, "Maximum smoothing duration"),
        BT::InputPort<bool>(
          "check_for_collisions", false,
          "If true collision check will be performed after smoothing"),
        BT::InputPort<std::string>("smoother_id", ""),
        BT::OutputPort<nav_msgs::msg::Path>(
          "smoothed_path",
          "Path smoothed by SmootherServer node"),
        BT::OutputPort<double>("smoothing_duration", "Time taken to smooth path"),
        BT::OutputPort<bool>(
          "was_completed", "True if smoothing was not interrupted by time limit"),
        BT::OutputPort<ActionResult::_error_code_type>(
          "error_code_id", "The smooth path error code"),
        BT::OutputPort<std::string>(
          "error_msg", "The smooth path error msg"),
      });
  }
};

}  // namespace nav3d_behavior_tree