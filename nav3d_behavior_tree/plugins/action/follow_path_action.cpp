#include <memory>
#include <string>

#include "nav3d_behavior_tree/plugins/action/follow_path_action.hpp"

namespace nav3d_behavior_tree
{

FollowPathAction::FollowPathAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<Action>(xml_tag_name, action_name, conf)
{
}

void FollowPathAction::on_tick()
{
  getInput("path", goal_.path);
  getInput("controller_id", goal_.controller_id);
  getInput("goal_checker_id", goal_.goal_checker_id);
  getInput("progress_checker_id", goal_.progress_checker_id);
}

BT::NodeStatus FollowPathAction::on_success()
{
  setOutput("error_code_id", ActionResult::NONE);
  setOutput("error_msg", "");
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus FollowPathAction::on_aborted()
{
  setOutput("error_code_id", result_.result->error_code);
  setOutput("error_msg", result_.result->error_msg);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus FollowPathAction::on_cancelled()
{
  setOutput("error_code_id", ActionResult::NONE);
  setOutput("error_msg", "");
  return BT::NodeStatus::SUCCESS;
}

void FollowPathAction::on_timeout()
{
  setOutput("error_code_id", ActionResult::CONTROLLER_TIMED_OUT);
  setOutput("error_msg", "Behavior Tree action client timed out waiting.");
}

void FollowPathAction::on_wait_for_result(
  std::shared_ptr<const Action::Feedback> /*feedback*/)
{
  nav_msgs::msg::Path new_path;
  getInput("path", new_path);

  if (goal_.path != new_path && new_path != nav_msgs::msg::Path()) {
    goal_.path = new_path;
    goal_updated_ = true;
  }

  std::string new_controller_id;
  getInput("controller_id", new_controller_id);
  if (goal_.controller_id != new_controller_id) {
    goal_.controller_id = new_controller_id;
    goal_updated_ = true;
  }

  std::string new_goal_checker_id;
  getInput("goal_checker_id", new_goal_checker_id);
  if (goal_.goal_checker_id != new_goal_checker_id) {
    goal_.goal_checker_id = new_goal_checker_id;
    goal_updated_ = true;
  }

  std::string new_progress_checker_id;
  getInput("progress_checker_id", new_progress_checker_id);
  if (goal_.progress_checker_id != new_progress_checker_id) {
    goal_.progress_checker_id = new_progress_checker_id;
    goal_updated_ = true;
  }
}

}  // namespace nav3d_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav3d_behavior_tree::FollowPathAction>(
        name, "follow_path", config);
    };

  factory.registerBuilder<nav3d_behavior_tree::FollowPathAction>(
    "FollowPath", builder);
}