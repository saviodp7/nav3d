#include "nav3d_planner_plugin_moveit/movegroup_interface.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/create_timer_ros.h"
#include "trajectory_msgs/msg/multi_dof_joint_trajectory.hpp"

namespace nav3d_planner_plugin_moveit
{

MoveGroupInterface::MoveGroupInterface(
  rclcpp_lifecycle::LifecycleNode::SharedPtr parent,
  std::string planning_group)
: parent_(std::move(parent)),
  planning_group_(std::move(planning_group))
{
  if (!parent_) {
    throw std::runtime_error("MoveGroupInterface: parent node is null");
  }

  rclcpp::NodeOptions opts;
  opts.context(parent_->get_node_base_interface()->get_context());

  node_ = std::make_shared<rclcpp::Node>(
    parent_->get_name() + std::string("_moveit"),
    parent_->get_namespace(),
    opts);

  logger_ = node_->get_logger();

  if (parent_->has_parameter("use_sim_time")) {
    const bool use_sim = parent_->get_parameter("use_sim_time").as_bool();
    if (!node_->has_parameter("use_sim_time")) {
      node_->declare_parameter("use_sim_time", use_sim);
    } else {
      node_->set_parameter(rclcpp::Parameter("use_sim_time", use_sim));
    }
  }

  startExecutor();

  move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(node_, planning_group_);
  move_group_->startStateMonitor();
  move_group_->setPlanningTime(planning_time_);
  move_group_->setNumPlanningAttempts(planning_attempts_);
}

MoveGroupInterface::~MoveGroupInterface()
{
  move_group_.reset();
  stopExecutor();
  node_.reset();
  parent_.reset();
}

void MoveGroupInterface::startExecutor()
{
  executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  spin_thread_ = std::thread([this]() { executor_->spin(); });
}

void MoveGroupInterface::stopExecutor()
{
  if (executor_) {
    executor_->cancel();
  }
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
  if (executor_ && node_) {
    executor_->remove_node(node_);
  }
  executor_.reset();
}

void MoveGroupInterface::setWorkspaceConstraints(
  double min_x, double min_y, double min_z,
  double max_x, double max_y, double max_z)
{
  move_group_->setWorkspace(min_x, min_y, min_z, max_x, max_y, max_z);
}

void MoveGroupInterface::setPlanningParameters(double planning_time, int attempts)
{
  planning_time_ = planning_time;
  planning_attempts_ = attempts;
  move_group_->setPlanningTime(planning_time_);
  move_group_->setNumPlanningAttempts(planning_attempts_);
}

void MoveGroupInterface::setPlannerIds(std::string pipeline_id, std::string planner_id)
{
  pipeline_id_ = std::move(pipeline_id);
  planner_id_ = std::move(planner_id);

  if (!planner_id_.empty()) {
    move_group_->setPlannerId(planner_id_);
  }
}

void MoveGroupInterface::setYawMode(YawMode mode, double fixed_yaw)
{
  yaw_mode_ = mode;
  fixed_yaw_ = fixed_yaw;
}

std::vector<geometry_msgs::msg::PoseStamped> MoveGroupInterface::planToGoalOnly(
  const geometry_msgs::msg::PoseStamped & goal,
  const std::function<bool()> & cancel_checker)
{
  if (cancel_checker && cancel_checker()) {
    return {};
  }

  std::map<std::string, double> goal_joints;
  const std::string p = "virtual_joint/";
  goal_joints[p + "trans_x"] = goal.pose.position.x;
  goal_joints[p + "trans_y"] = goal.pose.position.y;
  goal_joints[p + "trans_z"] = goal.pose.position.z;
  goal_joints[p + "rot_x"] = goal.pose.orientation.x;
  goal_joints[p + "rot_y"] = goal.pose.orientation.y;
  goal_joints[p + "rot_z"] = goal.pose.orientation.z;
  goal_joints[p + "rot_w"] = goal.pose.orientation.w;

  move_group_->setJointValueTarget(goal_joints);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const auto result = move_group_->plan(plan);

  if (cancel_checker && cancel_checker()) {
    return {};
  }

  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    return {};
  }

  const auto & traj = plan.trajectory_.multi_dof_joint_trajectory;
  const auto & points = traj.points;
  if (points.empty()) {
    return {};
  }

  std::vector<geometry_msgs::msg::PoseStamped> poses;
  poses.reserve(points.size());

  const auto frame_id =
    goal.header.frame_id.empty() ? move_group_->getPlanningFrame() : goal.header.frame_id;

  const auto stamp = node_->now();

  for (const auto & pt : points) {
    if (cancel_checker && cancel_checker()) {
      return {};
    }

    if (pt.transforms.empty()) {
      continue;
    }

    const auto & t = pt.transforms[0];

    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = stamp;
    ps.header.frame_id = frame_id;

    ps.pose.position.x = t.translation.x;
    ps.pose.position.y = t.translation.y;
    ps.pose.position.z = t.translation.z;

    ps.pose.orientation.x = t.rotation.x;
    ps.pose.orientation.y = t.rotation.y;
    ps.pose.orientation.z = t.rotation.z;
    ps.pose.orientation.w = t.rotation.w;

    poses.push_back(ps);
  }

  applyYaw(poses);
  return poses;
}

void MoveGroupInterface::applyYaw(std::vector<geometry_msgs::msg::PoseStamped> & poses) const
{
  if (poses.empty() || yaw_mode_ == YawMode::Keep) {
    return;
  }

  if (yaw_mode_ == YawMode::Fixed) {
    const auto q = quaternionFromYaw(fixed_yaw_);
    for (auto & p : poses) {
      p.pose.orientation = q;
    }
    return;
  }

  for (size_t i = 0; i + 1 < poses.size(); ++i) {
    const double yaw = computeDirectionalYaw(poses[i], poses[i + 1]);
    poses[i].pose.orientation = quaternionFromYaw(yaw);
  }

  poses.back().pose.orientation =
    poses.size() >= 2 ? poses[poses.size() - 2].pose.orientation : quaternionFromYaw(0.0);
}

double MoveGroupInterface::computeDirectionalYaw(
  const geometry_msgs::msg::PoseStamped & a,
  const geometry_msgs::msg::PoseStamped & b)
{
  const double dx = b.pose.position.x - a.pose.position.x;
  const double dy = b.pose.position.y - a.pose.position.y;

  if (std::hypot(dx, dy) < 1e-3) {
    return 0.0;
  }

  return std::atan2(dy, dx);
}

geometry_msgs::msg::Quaternion MoveGroupInterface::quaternionFromYaw(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  q.normalize();
  return tf2::toMsg(q);
}

}  // namespace nav3d_planner_plugin_moveit