#include <string>
#include <memory>
#include <vector>

#include "nav3d_util/geometry_utils.hpp"

#include "nav3d_behavior_tree/plugins/decorator/path_longer_on_approach.hpp"

namespace nav3d_behavior_tree
{

PathLongerOnApproach::PathLongerOnApproach(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

bool PathLongerOnApproach::isPathUpdated(
  nav_msgs::msg::Path & new_path,
  nav_msgs::msg::Path & old_path)
{
  return old_path.poses.size() != 0 &&
         new_path.poses.size() != 0 &&
         new_path.poses.size() != old_path.poses.size() &&
         old_path.poses.back().pose.position == new_path.poses.back().pose.position;
}

bool PathLongerOnApproach::isRobotInGoalProximity(
  nav_msgs::msg::Path & old_path,
  double & prox_leng)
{
  return nav3d_util::geometry_utils::calculate_path_length(old_path, 0) < prox_leng;
}

bool PathLongerOnApproach::isNewPathLonger(
  nav_msgs::msg::Path & new_path,
  nav_msgs::msg::Path & old_path,
  double & length_factor)
{
  return nav3d_util::geometry_utils::calculate_path_length(new_path, 0) >
         length_factor * nav3d_util::geometry_utils::calculate_path_length(
    old_path, 0);
}

inline BT::NodeStatus PathLongerOnApproach::tick()
{
  getInput("path", new_path_);
  getInput("prox_len", prox_len_);
  getInput("length_factor", length_factor_);

  if (first_time_ == false) {
    if (old_path_.poses.empty() || new_path_.poses.empty() ||
      old_path_.poses.back().pose != new_path_.poses.back().pose)
    {
      first_time_ = true;
    }
  }
  setStatus(BT::NodeStatus::RUNNING);

  if (isPathUpdated(new_path_, old_path_) && isRobotInGoalProximity(old_path_, prox_len_) &&
    isNewPathLonger(new_path_, old_path_, length_factor_) && !first_time_)
  {
    const BT::NodeStatus child_state = child_node_->executeTick();
    switch (child_state) {
      case BT::NodeStatus::SKIPPED:
      case BT::NodeStatus::RUNNING:
        return child_state;
      case BT::NodeStatus::SUCCESS:
      case BT::NodeStatus::FAILURE:
        old_path_ = new_path_;
        resetChild();
        return child_state;
      default:
        old_path_ = new_path_;
        return BT::NodeStatus::FAILURE;
    }
  }
  old_path_ = new_path_;
  first_time_ = false;
  return BT::NodeStatus::SUCCESS;
}

}

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav3d_behavior_tree::PathLongerOnApproach>("PathLongerOnApproach");
}