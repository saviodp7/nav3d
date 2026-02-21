#include <cmath>
#include <memory>
#include <string>

#include "behaviortree_cpp/decorator_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav3d_behavior_tree/plugins/action/truncate_path_action.hpp"
#include "nav3d_util/geometry_utils.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav3d_behavior_tree
{

TruncatePath::TruncatePath(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf),
  distance_(1.0)
{
}

BT::NodeStatus TruncatePath::tick()
{
  setStatus(BT::NodeStatus::RUNNING);
  getInput("distance", distance_);

  nav_msgs::msg::Path input_path;
  getInput("input_path", input_path);

  if (input_path.poses.empty()) {
    setOutput("output_path", input_path);
    return BT::NodeStatus::SUCCESS;
  }

  const geometry_msgs::msg::PoseStamped final_pose = input_path.poses.back();

  double distance_to_goal = nav3d_util::geometry_utils::euclidean_distance(
    input_path.poses.back(), final_pose);

  while (distance_to_goal < distance_ && input_path.poses.size() > 2) {
    input_path.poses.pop_back();
    distance_to_goal = nav3d_util::geometry_utils::euclidean_distance(
      input_path.poses.back(), final_pose);
  }

  const double dx = final_pose.pose.position.x - input_path.poses.back().pose.position.x;
  const double dy = final_pose.pose.position.y - input_path.poses.back().pose.position.y;

  double final_angle = std::atan2(dy, dx);

  if (std::isnan(final_angle) || std::isinf(final_angle)) {
    RCLCPP_WARN(
      config().blackboard->get<rclcpp::Node::SharedPtr>("node")->get_logger(),
      "Final angle is not valid while truncating path. Setting to 0.0");
    final_angle = 0.0;
  }

  input_path.poses.back().pose.orientation =
    nav3d_util::geometry_utils::orientationAroundZAxis(final_angle);

  setOutput("output_path", input_path);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav3d_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav3d_behavior_tree::TruncatePath>("TruncatePath");
}