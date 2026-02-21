#include <memory>
#include <string>
#include <functional>

#include "nav3d_behavior_tree/plugins/action/goal_checker_selector_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace nav3d_behavior_tree
{

using std::placeholders::_1;

GoalCheckerSelector::GoalCheckerSelector(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  initialize();
}

void GoalCheckerSelector::initialize()
{
  createROSInterfaces();
}

void GoalCheckerSelector::createROSInterfaces()
{
  std::string topic_new;
  getInput("topic_name", topic_new);
  if (topic_new != topic_name_ || !goal_checker_selector_sub_) {
    topic_name_ = topic_new;
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.transient_local().reliable();

    goal_checker_selector_sub_ = node_->create_subscription<std_msgs::msg::String>(
      topic_name_, qos, std::bind(&GoalCheckerSelector::callbackGoalCheckerSelect, this, _1));
  }
}

BT::NodeStatus GoalCheckerSelector::tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
  }

  rclcpp::spin_some(node_);

  if (last_selected_goal_checker_.empty()) {
    std::string default_goal_checker;
    getInput("default_goal_checker", default_goal_checker);
    if (default_goal_checker.empty()) {
      return BT::NodeStatus::FAILURE;
    }
    last_selected_goal_checker_ = default_goal_checker;
  }

  setOutput("selected_goal_checker", last_selected_goal_checker_);
  return BT::NodeStatus::SUCCESS;
}

void GoalCheckerSelector::callbackGoalCheckerSelect(const std_msgs::msg::String::SharedPtr msg)
{
  last_selected_goal_checker_ = msg->data;
}

}  // namespace nav3d_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav3d_behavior_tree::GoalCheckerSelector>("GoalCheckerSelector");
}