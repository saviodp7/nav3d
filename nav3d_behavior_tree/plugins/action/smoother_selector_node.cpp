#include <memory>
#include <string>
#include <functional>

#include "nav3d_behavior_tree/plugins/action/smoother_selector_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace nav3d_behavior_tree
{

using std::placeholders::_1;

SmootherSelector::SmootherSelector(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  initialize();
  callback_group_executor_.spin_some(std::chrono::nanoseconds(1));
}

void SmootherSelector::initialize()
{
  createROSInterfaces();
}

void SmootherSelector::createROSInterfaces()
{
  std::string topic_new;
  getInput("topic_name", topic_new);
  if (topic_new != topic_name_ || !smoother_selector_sub_) {
    topic_name_ = topic_new;
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive,
      false);
    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.transient_local().reliable();

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = callback_group_;
    smoother_selector_sub_ = node_->create_subscription<std_msgs::msg::String>(
      topic_name_,
      qos,
      std::bind(&SmootherSelector::callbackSmootherSelect, this, _1),
      sub_option);
  }
}

BT::NodeStatus SmootherSelector::tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
  }

  callback_group_executor_.spin_some();

  if (last_selected_smoother_.empty()) {
    std::string default_smoother;
    getInput("default_smoother", default_smoother);
    if (default_smoother.empty()) {
      return BT::NodeStatus::FAILURE;
    }
    last_selected_smoother_ = default_smoother;
  }

  setOutput("selected_smoother", last_selected_smoother_);
  return BT::NodeStatus::SUCCESS;
}

void SmootherSelector::callbackSmootherSelect(const std_msgs::msg::String::SharedPtr msg)
{
  last_selected_smoother_ = msg->data;
}

}  // namespace nav3d_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav3d_behavior_tree::SmootherSelector>("SmootherSelector");
}