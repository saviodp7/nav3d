#pragma once

#include <memory>
#include <string>

#include "nav3d_behavior_tree/bt_action_node.hpp"
#include "nav3d_msgs/action/follow_path.hpp"

namespace nav3d_behavior_tree
{

class FollowPathAction : public BtActionNode<nav3d_msgs::action::FollowPath>
{
  using Action = nav3d_msgs::action::FollowPath;
  using ActionResult = Action::Result;

public:
  FollowPathAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  BT::NodeStatus on_success() override;
  BT::NodeStatus on_aborted() override;
  BT::NodeStatus on_cancelled() override;
  void on_timeout() override;

  void on_wait_for_result(
    std::shared_ptr<const Action::Feedback> feedback) override;

  static BT::PortsList providedPorts()
  {
    BT::RegisterJsonDefinition<nav_msgs::msg::Path>();

    return providedBasicPorts(
      {
        BT::InputPort<nav_msgs::msg::Path>("path", "Path to follow"),
        BT::InputPort<std::string>("controller_id", ""),
        BT::InputPort<std::string>("goal_checker_id", ""),
        BT::InputPort<std::string>("progress_checker_id", ""),
        BT::OutputPort<ActionResult::_error_code_type>(
          "error_code_id", "The follow path error code"),
        BT::OutputPort<std::string>(
          "error_msg", "The follow path error msg"),
      });
  }
};

}  // namespace nav3d_behavior_tree