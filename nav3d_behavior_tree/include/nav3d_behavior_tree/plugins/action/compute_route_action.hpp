#pragma once

#include <string>

#include "nav3d_behavior_tree/bt_action_node.hpp"
#include "nav3d_msgs/action/compute_route.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav3d_behavior_tree
{

class ComputeRouteAction : public BtActionNode<nav3d_msgs::action::ComputeRoute>
{
  using Action = nav3d_msgs::action::ComputeRoute;
  using ActionResult = Action::Result;

public:
  ComputeRouteAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  BT::NodeStatus on_success() override;
  BT::NodeStatus on_aborted() override;
  BT::NodeStatus on_cancelled() override;
  void on_timeout() override;

  void halt() override;

  void resetPorts();

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<unsigned int>("start_id", "ID of the start node"),
        BT::InputPort<unsigned int>("goal_id", "ID of the goal node"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>(
          "start",
          "Start pose of the path if overriding current robot pose and using poses over IDs"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>(
          "goal", "Goal pose of the path if using poses over IDs"),
        BT::InputPort<bool>(
          "use_start", false,
          "Whether to use the start pose or the robot's current pose"),
        BT::InputPort<bool>(
          "use_poses", false, "Whether to use poses or IDs for start and goal"),
        BT::OutputPort<ActionResult::_route_type>(
          "route", "The route computed by ComputeRoute node"),
        BT::OutputPort<builtin_interfaces::msg::Duration>(
          "planning_time",
          "Time taken to compute route"),
        BT::OutputPort<nav_msgs::msg::Path>("path", "Path created by ComputeRoute node"),
        BT::OutputPort<ActionResult::_error_code_type>(
          "error_code_id", "The compute route error code"),
        BT::OutputPort<std::string>(
          "error_msg", "The compute route error msg"),
      });
  }
};

}  // namespace nav3d_behavior_tree