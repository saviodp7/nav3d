#include <memory>
#include <string>
#include <vector>

#include "nav3d_behavior_tree/plugins/action/compute_path_through_poses_action.hpp"

namespace nav3d_behavior_tree
{

ComputePathThroughPosesAction::ComputePathThroughPosesAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav3d_msgs::action::ComputePathThroughPoses>(xml_tag_name, action_name, conf)
{
}

void ComputePathThroughPosesAction::on_tick()
{
  getInput("goals", goal_.goals);
  getInput("planner_id", goal_.planner_id);
  goal_.use_start = false;
  if (getInput("start", goal_.start)) {
    goal_.use_start = true;
  }
}

BT::NodeStatus ComputePathThroughPosesAction::on_success()
{
  setOutput("path", result_.result->path);
  setOutput("error_code_id", ActionResult::NONE);
  setOutput("error_msg", "");
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ComputePathThroughPosesAction::on_aborted()
{
  nav_msgs::msg::Path empty_path;
  setOutput("path", empty_path);
  setOutput("error_code_id", result_.result->error_code);
  setOutput("error_msg", result_.result->error_msg);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ComputePathThroughPosesAction::on_cancelled()
{
  nav_msgs::msg::Path empty_path;
  setOutput("path", empty_path);
  setOutput("error_code_id", ActionResult::NONE);
  setOutput("error_msg", "");
  return BT::NodeStatus::SUCCESS;
}

void ComputePathThroughPosesAction::on_timeout()
{
  setOutput("error_code_id", ActionResult::TIMEOUT);
  setOutput("error_msg", "Behavior Tree action client timed out waiting.");
}

void ComputePathThroughPosesAction::halt()
{
  nav_msgs::msg::Path empty_path;
  setOutput("path", empty_path);
  BtActionNode::halt();
}

}  // namespace nav3d_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav3d_behavior_tree::ComputePathThroughPosesAction>(
        name, "compute_path_through_poses", config);
    };

  factory.registerBuilder<nav3d_behavior_tree::ComputePathThroughPosesAction>(
    "ComputePathThroughPoses", builder);
}