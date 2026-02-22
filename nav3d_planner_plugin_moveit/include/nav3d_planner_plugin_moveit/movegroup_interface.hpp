#pragma once

#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace nav3d_planner_plugin_moveit
{

enum class YawMode
{
  Keep,
  Directional,
  Fixed
};

class MoveGroupInterface
{
public:
  explicit MoveGroupInterface(
    rclcpp_lifecycle::LifecycleNode::SharedPtr parent,
    std::string planning_group);

  ~MoveGroupInterface();

  void setWorkspaceConstraints(
    double min_x, double min_y, double min_z,
    double max_x, double max_y, double max_z);

  void setPlanningParameters(double planning_time, int attempts);

  void setPlannerIds(std::string pipeline_id, std::string planner_id);

  void setYawMode(YawMode mode, double fixed_yaw);

  std::vector<geometry_msgs::msg::PoseStamped> planToGoalOnly(
    const geometry_msgs::msg::PoseStamped & goal,
    const std::function<bool()> & cancel_checker);

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr parent_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("nav3d_planner_plugin_moveit.movegroup_interface")};

  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;

  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::string planning_group_;

  double planning_time_{10.0};
  int planning_attempts_{3};

  std::string pipeline_id_{};
  std::string planner_id_{};

  YawMode yaw_mode_{YawMode::Keep};
  double fixed_yaw_{0.0};

  void startExecutor();
  void stopExecutor();

  void applyYaw(std::vector<geometry_msgs::msg::PoseStamped> & poses) const;

  static double computeDirectionalYaw(
    const geometry_msgs::msg::PoseStamped & a,
    const geometry_msgs::msg::PoseStamped & b);

  static geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw);
};

}  // namespace nav3d_planner_plugin_moveit