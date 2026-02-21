#include <memory>
#include <string>

#include "nav3d_behavior_tree/plugins/action/concatenate_paths_action.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

namespace nav3d_behavior_tree
{

ConcatenatePaths::ConcatenatePaths(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
}

BT::NodeStatus ConcatenatePaths::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  nav_msgs::msg::Path input_path1, input_path2;
  getInput("input_path1", input_path1);
  getInput("input_path2", input_path2);

  if (input_path1.poses.empty() && input_path2.poses.empty()) {
    RCLCPP_ERROR(
      config().blackboard->get<rclcpp::Node::SharedPtr>("node")->get_logger(),
      "No input paths provided to concatenate. Both paths are empty.");
    return BT::NodeStatus::FAILURE;
  }

  nav_msgs::msg::Path output_path = input_path1;
  if (input_path1.header != std_msgs::msg::Header()) {
    output_path.header = input_path1.header;
  } else if (input_path2.header != std_msgs::msg::Header()) {
    output_path.header = input_path2.header;
  }

  output_path.poses.insert(
    output_path.poses.end(),
    input_path2.poses.begin(),
    input_path2.poses.end());

  setOutput("output_path", output_path);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav3d_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav3d_behavior_tree::ConcatenatePaths>("ConcatenatePaths");
}