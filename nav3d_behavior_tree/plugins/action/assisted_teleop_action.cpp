#include <memory>
#include <string>

#include "nav3d_behavior_tree/plugins/action/assisted_teleop_action.hpp"

namespace nav3d_behavior_tree
{

AssistedTeleopAction::AssistedTeleopAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav3d_msgs::action::AssistedTeleop>(xml_tag_name, action_name, conf)
{
}

void AssistedTeleopAction::initialize()
{
  double time_allowance;
  getInput("time_allowance", time_allowance);
  getInput("is_recovery", is_recovery_);

  goal_.time_allowance = rclcpp::Duration::from_seconds(time_allowance);
}

void AssistedTeleopAction::on_tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
  }

  if (is_recovery_) {
    increment_recovery_count();
  }
}

BT::NodeStatus AssistedTeleopAction::on_success()
{
  setOutput("error_code_id", ActionResult::NONE);
  setOutput("error_msg", "");
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus AssistedTeleopAction::on_aborted()
{
  setOutput("error_code_id", result_.result->error_code);
  setOutput("error_msg", result_.result->error_msg);
  return is_recovery_ ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
}

BT::NodeStatus AssistedTeleopAction::on_cancelled()
{
  setOutput("error_code_id", ActionResult::NONE);
  setOutput("error_msg", "");
  return BT::NodeStatus::SUCCESS;
}

void AssistedTeleopAction::on_timeout()
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
      return std::make_unique<nav3d_behavior_tree::AssistedTeleopAction>(
        name, "assisted_teleop", config);
    };

  factory.registerBuilder<nav3d_behavior_tree::AssistedTeleopAction>("AssistedTeleop", builder);
}