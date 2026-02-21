#include <memory>
#include <string>

#include "nav3d_behavior_tree/plugins/action/back_up_action.hpp"

namespace nav3d_behavior_tree
{

BackUpAction::BackUpAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav3d_msgs::action::BackUp>(xml_tag_name, action_name, conf)
{
}

void BackUpAction::initialize()
{
  double dist;
  getInput("backup_dist", dist);
  double speed;
  getInput("backup_speed", speed);
  double time_allowance;
  getInput("time_allowance", time_allowance);
  bool disable_collision_checks;
  getInput("disable_collision_checks", disable_collision_checks);

  goal_.target.x = dist;
  goal_.target.y = 0.0;
  goal_.target.z = 0.0;
  goal_.speed = speed;
  goal_.time_allowance = rclcpp::Duration::from_seconds(time_allowance);
  goal_.disable_collision_checks = disable_collision_checks;
}

void BackUpAction::on_tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
  }

  increment_recovery_count();
}

BT::NodeStatus BackUpAction::on_success()
{
  setOutput("error_code_id", ActionResult::NONE);
  setOutput("error_msg", "");
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus BackUpAction::on_aborted()
{
  setOutput("error_code_id", result_.result->error_code);
  setOutput("error_msg", result_.result->error_msg);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus BackUpAction::on_cancelled()
{
  setOutput("error_code_id", ActionResult::NONE);
  setOutput("error_msg", "");
  return BT::NodeStatus::SUCCESS;
}

void BackUpAction::on_timeout()
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
      return std::make_unique<nav3d_behavior_tree::BackUpAction>(
        name, "backup", config);
    };

  factory.registerBuilder<nav3d_behavior_tree::BackUpAction>("BackUp", builder);
}