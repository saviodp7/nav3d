#include <memory>
#include <string>

#include "nav3d_behavior_tree/plugins/action/smooth_path_action.hpp"

namespace nav3d_behavior_tree
{

SmoothPathAction::SmoothPathAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav3d_msgs::action::SmoothPath>(xml_tag_name, action_name, conf)
{
}

void SmoothPathAction::on_tick()
{
  getInput("unsmoothed_path", goal_.path);
  getInput("smoother_id", goal_.smoother_id);

  double max_smoothing_duration = 0.0;
  getInput("max_smoothing_duration", max_smoothing_duration);
  goal_.max_smoothing_duration = rclcpp::Duration::from_seconds(max_smoothing_duration);

  getInput("check_for_collisions", goal_.check_for_collisions);
}

BT::NodeStatus SmoothPathAction::on_success()
{
  setOutput("smoothed_path", result_.result->path);
  setOutput("smoothing_duration", rclcpp::Duration(result_.result->smoothing_duration).seconds());
  setOutput("was_completed", result_.result->was_completed);
  setOutput("error_code_id", ActionResult::NONE);
  setOutput("error_msg", "");
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SmoothPathAction::on_aborted()
{
  setOutput("error_code_id", result_.result->error_code);
  setOutput("error_msg", result_.result->error_msg);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus SmoothPathAction::on_cancelled()
{
  setOutput("error_code_id", ActionResult::NONE);
  setOutput("error_msg", "");
  return BT::NodeStatus::SUCCESS;
}

void SmoothPathAction::on_timeout()
{
  setOutput("error_code_id", ActionResult::TIMEOUT);
  setOutput("error_msg", "Behavior Tree action client timed out waiting.");
}

}  // namespace nav3d_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav3d_behavior_tree::SmoothPathAction>(
        name, "smooth_path", config);
    };

  factory.registerBuilder<nav3d_behavior_tree::SmoothPathAction>(
    "SmoothPath", builder);
}