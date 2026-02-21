#include <memory>
#include <string>
#include <functional>

#include "nav3d_behavior_tree/plugins/action/progress_checker_selector_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace nav3d_behavior_tree
{

using std::placeholders::_1;

ProgressCheckerSelector::ProgressCheckerSelector(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  initialize();
}

void ProgressCheckerSelector::initialize()
{
  createROSInterfaces();
}

void ProgressCheckerSelector::createROSInterfaces()
{
  std::string topic_new;
  getInput("topic_name", topic_new);
  if (topic_new != topic_name_ || !progress_checker_selector_sub_) {
    topic_name_ = topic_new;
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.transient_local().reliable();

    progress_checker_selector_sub_ = node_->create_subscription<std_msgs::msg::String>(
      topic_name_,
      qos,
      std::bind(&ProgressCheckerSelector::callbackProgressCheckerSelect, this, _1));
  }
}

BT::NodeStatus ProgressCheckerSelector::tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
  }

  rclcpp::spin_some(node_);

  if (last_selected_progress_checker_.empty()) {
    std::string default_progress_checker;
    getInput("default_progress_checker", default_progress_checker);
    if (default_progress_checker.empty()) {
      return BT::NodeStatus::FAILURE;
    }
    last_selected_progress_checker_ = default_progress_checker;
  }

  setOutput("selected_progress_checker", last_selected_progress_checker_);
  return BT::NodeStatus::SUCCESS;
}

void ProgressCheckerSelector::callbackProgressCheckerSelect(const std_msgs::msg::String::SharedPtr msg)
{
  last_selected_progress_checker_ = msg->data;
}

}  // namespace nav3d_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav3d_behavior_tree::ProgressCheckerSelector>("ProgressCheckerSelector");
}