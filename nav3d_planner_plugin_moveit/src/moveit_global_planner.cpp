#include "nav3d_planner_plugin_moveit/moveit_global_planner.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>
#include <algorithm>

#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav3d_planner_plugin_moveit
{

static bool isFinitePose(const geometry_msgs::msg::PoseStamped & p)
{
  const auto & pos = p.pose.position;
  const auto & ori = p.pose.orientation;

  const auto finite = [](double v) { return std::isfinite(v); };

  return finite(pos.x) && finite(pos.y) && finite(pos.z) &&
         finite(ori.x) && finite(ori.y) && finite(ori.z) && finite(ori.w);
}

static bool isQuaternionReasonablyNormalized(const geometry_msgs::msg::Quaternion & q)
{
  const double n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  return std::abs(n - 1.0) < 0.1;
}

void MoveItGlobalPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf)
{
  node_ = parent.lock();
  if (!node_) {
    throw std::runtime_error("MoveItGlobalPlanner: parent node expired");
  }

  name_ = std::move(name);
  tf_ = std::move(tf);
  logger_ = node_->get_logger();

  if (!node_->has_parameter(name_ + ".planning_group")) {
    node_->declare_parameter(name_ + ".planning_group", planning_group_);
  }
  if (!node_->has_parameter(name_ + ".planning_time")) {
    node_->declare_parameter(name_ + ".planning_time", planning_time_);
  }
  if (!node_->has_parameter(name_ + ".planning_attempts")) {
    node_->declare_parameter(name_ + ".planning_attempts", planning_attempts_);
  }

  if (!node_->has_parameter(name_ + ".global_frame")) {
    node_->declare_parameter(name_ + ".global_frame", global_frame_);
  }
  if (!node_->has_parameter(name_ + ".transform_tolerance")) {
    node_->declare_parameter(name_ + ".transform_tolerance", transform_tolerance_);
  }
  if (!node_->has_parameter(name_ + ".pipeline_id")) {
    node_->declare_parameter(name_ + ".pipeline_id", std::string(""));
  }
  if (!node_->has_parameter(name_ + ".planner_id")) {
    node_->declare_parameter(name_ + ".planner_id", std::string(""));
  }
  if (!node_->has_parameter(name_ + ".yaw_mode")) {
    node_->declare_parameter(name_ + ".yaw_mode", std::string("directional"));
  }
  if (!node_->has_parameter(name_ + ".fixed_yaw")) {
    node_->declare_parameter(name_ + ".fixed_yaw", 0.0);
  }

  if (!node_->has_parameter(name_ + ".validity.step")) {
    node_->declare_parameter(name_ + ".validity.step", validity_step_);
  }
  if (!node_->has_parameter(name_ + ".validity.max_checks")) {
    node_->declare_parameter(name_ + ".validity.max_checks", validity_max_checks_);
  }
  if (!node_->has_parameter(name_ + ".validity.timeout_ms")) {
    node_->declare_parameter(name_ + ".validity.timeout_ms", validity_timeout_ms_);
  }
  if (!node_->has_parameter(name_ + ".validity.state_validity_service")) {
    node_->declare_parameter(name_ + ".validity.state_validity_service", state_validity_service_name_);
  }

  if (!node_->has_parameter(name_ + ".workspace.min_x")) {
    node_->declare_parameter(name_ + ".workspace.min_x", -0.5);
  }
  if (!node_->has_parameter(name_ + ".workspace.min_y")) {
    node_->declare_parameter(name_ + ".workspace.min_y", -0.5);
  }
  if (!node_->has_parameter(name_ + ".workspace.min_z")) {
    node_->declare_parameter(name_ + ".workspace.min_z", -0.1);
  }
  if (!node_->has_parameter(name_ + ".workspace.max_x")) {
    node_->declare_parameter(name_ + ".workspace.max_x", 20.0);
  }
  if (!node_->has_parameter(name_ + ".workspace.max_y")) {
    node_->declare_parameter(name_ + ".workspace.max_y", 10.0);
  }
  if (!node_->has_parameter(name_ + ".workspace.max_z")) {
    node_->declare_parameter(name_ + ".workspace.max_z", 5.0);
  }

  planning_group_ = node_->get_parameter(name_ + ".planning_group").as_string();
  planning_time_ = node_->get_parameter(name_ + ".planning_time").as_double();
  planning_attempts_ = static_cast<int>(node_->get_parameter(name_ + ".planning_attempts").as_int());

  global_frame_ = node_->get_parameter(name_ + ".global_frame").as_string();
  transform_tolerance_ = node_->get_parameter(name_ + ".transform_tolerance").as_double();

  const auto pipeline_id = node_->get_parameter(name_ + ".pipeline_id").as_string();
  const auto planner_id = node_->get_parameter(name_ + ".planner_id").as_string();

  const auto yaw_mode_s = node_->get_parameter(name_ + ".yaw_mode").as_string();
  const auto fixed_yaw = node_->get_parameter(name_ + ".fixed_yaw").as_double();

  const auto min_x = node_->get_parameter(name_ + ".workspace.min_x").as_double();
  const auto min_y = node_->get_parameter(name_ + ".workspace.min_y").as_double();
  const auto min_z = node_->get_parameter(name_ + ".workspace.min_z").as_double();
  const auto max_x = node_->get_parameter(name_ + ".workspace.max_x").as_double();
  const auto max_y = node_->get_parameter(name_ + ".workspace.max_y").as_double();
  const auto max_z = node_->get_parameter(name_ + ".workspace.max_z").as_double();

  validity_step_ = static_cast<int>(node_->get_parameter(name_ + ".validity.step").as_int());
  validity_max_checks_ = static_cast<int>(node_->get_parameter(name_ + ".validity.max_checks").as_int());
  validity_timeout_ms_ = static_cast<int>(node_->get_parameter(name_ + ".validity.timeout_ms").as_int());
  state_validity_service_name_ = node_->get_parameter(name_ + ".validity.state_validity_service").as_string();

  move_group_interface_ = std::make_unique<MoveGroupInterface>(node_, planning_group_);
  move_group_interface_->setStateValidityServiceName(state_validity_service_name_);
  move_group_interface_->setPlanningParameters(planning_time_, planning_attempts_);
  move_group_interface_->setPlannerIds(pipeline_id, planner_id);
  move_group_interface_->setWorkspaceConstraints(min_x, min_y, min_z, max_x, max_y, max_z);

  if (yaw_mode_s == "fixed") {
    move_group_interface_->setYawMode(YawMode::Fixed, fixed_yaw);
  } else if (yaw_mode_s == "directional") {
    move_group_interface_->setYawMode(YawMode::Directional, 0.0);
  } else {
    move_group_interface_->setYawMode(YawMode::Keep, 0.0);
  }

  active_ = false;
}

void MoveItGlobalPlanner::cleanup()
{
  move_group_interface_.reset();
  active_ = false;
}

void MoveItGlobalPlanner::activate()
{
  active_ = true;
}

void MoveItGlobalPlanner::deactivate()
{
  active_ = false;
}

bool MoveItGlobalPlanner::isPathValid(
  const nav_msgs::msg::Path & path,
  std::vector<int32_t> & invalid_pose_indices)
{
  invalid_pose_indices.clear();

  if (!active_ || !node_ || !move_group_interface_) {
    return true;
  }

  if (path.poses.empty()) {
    return true;
  }

  const auto timeout = std::chrono::milliseconds(std::max(0, validity_timeout_ms_));

  const int step = std::max(1, validity_step_);
  const int max_checks = std::max(1, validity_max_checks_);

  int checks = 0;
  for (size_t i = 0; i < path.poses.size(); i += static_cast<size_t>(step)) {
    if (++checks > max_checks) {
      break;
    }

    if (!move_group_interface_->isPoseValid(path.poses[i], timeout)) {
      invalid_pose_indices.push_back(static_cast<int32_t>(i));
      return false;
    }
  }

  return true;
}

bool MoveItGlobalPlanner::validatePose(const geometry_msgs::msg::PoseStamped & pose) const
{
  if (!isFinitePose(pose)) {
    return false;
  }
  if (!isQuaternionReasonablyNormalized(pose.pose.orientation)) {
    return false;
  }
  return true;
}

nav_msgs::msg::Path MoveItGlobalPlanner::toPath(
  const std::vector<geometry_msgs::msg::PoseStamped> & poses,
  const std::string & frame_id) const
{
  nav_msgs::msg::Path out;
  out.header.stamp = node_->now();
  out.header.frame_id = frame_id;
  out.poses = poses;

  for (auto & p : out.poses) {
    p.header.frame_id = frame_id;
    p.header.stamp = out.header.stamp;
  }

  return out;
}

nav_msgs::msg::Path MoveItGlobalPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped &,
  const geometry_msgs::msg::PoseStamped & goal_in,
  std::function<bool()> cancel_checker)
{
  nav_msgs::msg::Path empty;
  empty.header.stamp = node_ ? node_->now() : rclcpp::Time(0);
  empty.header.frame_id = global_frame_;

  if (!active_ || !node_ || !move_group_interface_) {
    return empty;
  }

  if (cancel_checker && cancel_checker()) {
    return empty;
  }

  if (!validatePose(goal_in)) {
    return empty;
  }

  geometry_msgs::msg::PoseStamped goal = goal_in;

  try {
    if (tf_) {
      const auto timeout = tf2::durationFromSec(transform_tolerance_);
      if (!goal.header.frame_id.empty() && goal.header.frame_id != global_frame_) {
        goal = tf_->transform(goal, global_frame_, timeout);
      }
    }
  } catch (const tf2::TransformException &) {
    return empty;
  }

  if (cancel_checker && cancel_checker()) {
    return empty;
  }

  const auto poses = move_group_interface_->plan(goal, cancel_checker);
  if (poses.empty()) {
    return empty;
  }

  return toPath(poses, global_frame_);
}

}  // namespace nav3d_planner_plugin_moveit

PLUGINLIB_EXPORT_CLASS(nav3d_planner_plugin_moveit::MoveItGlobalPlanner, nav3d_core::GlobalPlanner)