#include <cmath>
#include <memory>
#include <string>

#include "nav3d_behavior_tree/plugins/action/wait_action.hpp"

namespace nav3d_behavior_tree
{

WaitAction::WaitAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav3d_msgs::action::Wait>(xml_tag_name, action_name, conf)
{
}

void WaitAction::initialize()
{
  double duration = 0.0;
  getInput("wait_duration", duration);
  if (duration <= 0.0) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Wait duration is negative or zero (%f). Setting to positive.",
      duration);
    duration = std::abs(duration);
  }

  goal_.time = rclcpp::Duration::from_seconds(duration);
}

void WaitAction::on_tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
  }

  increment_recovery_count();
}

BT::NodeStatus WaitAction::on_success()
{
  setOutput("error_code_id", ActionResult::NONE);
  setOutput("error_msg", "");
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus WaitAction::on_aborted()
{
  setOutput("error_code_id", result_.result->error_code);
  setOutput("error_msg", result_.result->error_msg);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus WaitAction::on_cancelled()
{
  setOutput("error_code_id", ActionResult::NONE);
  setOutput("error_msg", "");
  return BT::NodeStatus::SUCCESS;
}

void WaitAction::on_timeout()
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
      return std::make_unique<nav3d_behavior_tree::WaitAction>(name, "wait", config);
    };

  factory.registerBuilder<nav3d_behavior_tree::WaitAction>("Wait", builder);
}