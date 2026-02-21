#include <string>
#include <memory>
#include <vector>

#include "nav3d_util/geometry_utils.hpp"

#include "nav3d_behavior_tree/plugins/decorator/speed_controller.hpp"

namespace nav3d_behavior_tree
{

SpeedController::SpeedController(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf),
  first_tick_(false),
  period_(1.0),
  min_rate_(0.1),
  max_rate_(1.0),
  min_speed_(0.0),
  max_speed_(0.5)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  getInput("min_rate", min_rate_);
  getInput("max_rate", max_rate_);
  getInput("min_speed", min_speed_);
  getInput("max_speed", max_speed_);

  if (min_rate_ <= 0.0 || max_rate_ <= 0.0) {
    std::string err_msg = "SpeedController node cannot have rate <= 0.0";
    RCLCPP_FATAL(node_->get_logger(), "%s", err_msg.c_str());
    throw BT::BehaviorTreeException(err_msg);
  }

  d_rate_ = max_rate_ - min_rate_;
  d_speed_ = max_speed_ - min_speed_;

  odom_smoother_ = config().blackboard->get<std::shared_ptr<nav3d_util::OdomSmoother>>(
    "odom_smoother");
}

inline BT::NodeStatus SpeedController::tick()
{
  if (!BT::isStatusActive(status())) {
    BT::getInputOrBlackboard("goals", goals_);
    BT::getInputOrBlackboard("goal", goal_);
    period_ = 1.0 / max_rate_;
    start_ = node_->now();
    first_tick_ = true;
  }

  nav_msgs::msg::Goals current_goals;
  BT::getInputOrBlackboard("goals", current_goals);
  geometry_msgs::msg::PoseStamped current_goal;
  BT::getInputOrBlackboard("goal", current_goal);

  if (goal_ != current_goal || goals_ != current_goals) {
    period_ = 1.0 / max_rate_;
    start_ = node_->now();
    first_tick_ = true;
    goal_ = current_goal;
    goals_ = current_goals;
  }

  setStatus(BT::NodeStatus::RUNNING);

  auto elapsed = node_->now() - start_;

  if (first_tick_ || (child_node_->status() == BT::NodeStatus::RUNNING) ||
    elapsed.seconds() >= period_)
  {
    first_tick_ = false;

    if (elapsed.seconds() >= period_) {
      updatePeriod();
      start_ = node_->now();
    }

    return child_node_->executeTick();
  }

  return status();
}

}

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav3d_behavior_tree::SpeedController>("SpeedController");
}