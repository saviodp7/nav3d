#include <memory>
#include <string>

#include "nav3d_behavior_tree/plugins/action/spin_action.hpp"

namespace nav3d_behavior_tree
{

SpinAction::SpinAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav3d_msgs::action::Spin>(xml_tag_name, action_name, conf)
{
}

void SpinAction::initialize()
{
  double dist = 0.0;
  getInput("spin_dist", dist);

  double time_allowance = 0.0;
  getInput("time_allowance", time_allowance);

  goal_.target_yaw = dist;
  goal_.time_allowance = rclcpp::Duration::from_seconds(time_allowance);

  getInput("is_recovery", is_recovery_);
}

void SpinAction::on_tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
  }

  if (is_recovery_) {
    increment_recovery_count();
  }
}

BT::NodeStatus SpinAction::on_success()
{
  setOutput("error_code_id", ActionResult::NONE);
  setOutput("error_msg", "");
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SpinAction::on_aborted()
{
  setOutput("error_code_id", result_.result->error_code);
  setOutput("error_msg", result_.result->error_msg);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus SpinAction::on_cancelled()
{
  setOutput("error_code_id", ActionResult::NONE);
  setOutput("error_msg", "");
  return BT::NodeStatus::SUCCESS;
}

void SpinAction::on_timeout()
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
      return std::make_unique<nav3d_behavior_tree::SpinAction>(name, "spin", config);
    };

  factory.registerBuilder<nav3d_behavior_tree::SpinAction>("Spin", builder);
}