#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

#include "nav3d_core/global_planner.hpp"
#include "nav3d_core/path_validity_checker.hpp"
#include "nav3d_planner_plugin_moveit/movegroup_interface.hpp"

namespace nav3d_planner_plugin_moveit
{

class MoveItGlobalPlanner final : public nav3d_core::GlobalPlanner, public nav3d_core::PathValidityChecker
{
public:
  MoveItGlobalPlanner() = default;
  ~MoveItGlobalPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  bool isPathValid(
    const nav_msgs::msg::Path & path,
    std::vector<int32_t> & invalid_pose_indices) override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("nav3d_planner_plugin_moveit")};
  std::string name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  std::unique_ptr<MoveGroupInterface> move_group_interface_;

  std::string planning_group_{"x500_group"};
  double planning_time_{10.0};
  int planning_attempts_{3};

  bool active_{false};

  int validity_step_{2};
  int validity_max_checks_{200};
  int validity_timeout_ms_{50};
  std::string state_validity_service_name_{"check_state_validity"};

  std::string global_frame_{"map"};
  double transform_tolerance_{0.1};

  bool validatePose(const geometry_msgs::msg::PoseStamped & pose) const;

  nav_msgs::msg::Path toPath(
    const std::vector<geometry_msgs::msg::PoseStamped> & poses,
    const std::string & frame_id) const;
};

}  // namespace nav3d_planner_plugin_moveit