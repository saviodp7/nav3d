#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "nav3d_core/global_planner.hpp"
#include "nav3d_core/planner_exceptions.hpp"
#include "nav3d_msgs/action/compute_path_to_pose.hpp"
#include "nav3d_util/lifecycle_node.hpp"
#include "nav3d_util/simple_action_server.hpp"

#include "pluginlib/class_loader.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace nav3d_planner
{

/**
 * @class nav3d_planner::PlannerServer
 * @brief Action server implementing ComputePathToPose and hosting GlobalPlanner plugins.
 *
 * Prima usavi un servizio esterno (x500_trajectory_planner). Ora l’implementazione di planning
 * vive nei plugin (es. nav3d_planner_plugin_moveit) e questo server si occupa solo di:
 * - gestione azione / cancellazione / preemption
 * - TF per ricavare start pose (se non fornita)
 * - selezione del plugin via planner_id
 * - pubblicazione del path per visualizzazione
 */
class PlannerServer : public nav3d_util::LifecycleNode
{
public:
  explicit PlannerServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~PlannerServer() override = default;

protected:
  nav3d_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  nav3d_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  nav3d_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  nav3d_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  nav3d_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  using ActionToPose = nav3d_msgs::action::ComputePathToPose;
  using ActionToPoseResult = ActionToPose::Result;
  using ActionServerToPose = nav3d_util::SimpleActionServer<ActionToPose>;

  template<typename T>
  bool isServerInactive(std::unique_ptr<nav3d_util::SimpleActionServer<T>> & action_server);

  template<typename T>
  bool isCancelRequested(std::unique_ptr<nav3d_util::SimpleActionServer<T>> & action_server);

  template<typename T>
  void getPreemptedGoalIfRequested(
    std::unique_ptr<nav3d_util::SimpleActionServer<T>> & action_server,
    typename std::shared_ptr<const typename T::Goal> & goal);

  void computePlan();

  void publishPlan(const nav_msgs::msg::Path & path);

private:
  pluginlib::ClassLoader<nav3d_core::GlobalPlanner> planner_loader_;
  std::vector<std::string> planner_ids_;
  std::unordered_map<std::string, nav3d_core::GlobalPlanner::Ptr> planners_;

  std::unique_ptr<ActionServerToPose> action_server_pose_;

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr plan_publisher_;

  std::string global_frame_{"map"};
  std::string robot_base_frame_{"base_link"};
  double transform_tolerance_{0.1};

  double expected_planner_frequency_{1.0};
  double action_server_result_timeout_{10.0};

  nav3d_core::GlobalPlanner::Ptr getPlannerOrThrow(const std::string & planner_id);
};

}  // namespace nav3d_planner