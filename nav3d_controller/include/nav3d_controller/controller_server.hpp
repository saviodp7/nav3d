#pragma once

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "angles/angles.h"
#include "tf2/utils.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "nav3d_core/controller.hpp"
#include "nav3d_msgs/action/follow_path.hpp"
#include "nav3d_msgs/msg/speed_limit.hpp"
#include "nav3d_util/lifecycle_node.hpp"
#include "nav3d_util/robot_utils.hpp"
#include "nav3d_util/simple_action_server.hpp"

#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_ros/transform_listener.h"

namespace nav3d_controller
{

class ControllerServer : public nav3d_util::LifecycleNode
{
public:
  using ControllerMap = std::unordered_map<std::string, nav3d_core::Controller::Ptr>;
  using Action = nav3d_msgs::action::FollowPath;
  using ActionServer = nav3d_util::SimpleActionServer<Action>;

  explicit ControllerServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ControllerServer();

protected:
  nav3d_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  nav3d_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  nav3d_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  nav3d_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  nav3d_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  std::unique_ptr<ActionServer> action_server_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_pub_;

  // FollowPath execution loop:
  // - converts the Path into a time-indexed trajectory (ENU)
  // - publishes the current waypoint on waypoint_topic_
  // - terminates on goal reached / cancel / error
  void computeControl();

  void generateTimedTrajectory(const nav_msgs::msg::Path & path);
  std::optional<geometry_msgs::msg::PoseStamped> getCurrentWaypoint();
  bool isGoalReached();

  rclcpp::Subscription<nav3d_msgs::msg::SpeedLimit>::SharedPtr speed_limit_sub_;
  void speedLimitCallback(const nav3d_msgs::msg::SpeedLimit::SharedPtr msg);

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string global_frame_{"map"};
  std::string robot_base_frame_{"base_link"};
  double transform_tolerance_{0.1};

  std::string waypoint_topic_{"offboard/waypoint"};
  double controller_frequency_{10.0};

  double xyz_goal_tolerance_{0.1};
  double yaw_goal_tolerance_{0.1};

  double speed_limit_{0.5};
  double default_speed_limit_{0.5};

  rclcpp::Time last_valid_cmd_time_;

  std::vector<std::pair<rclcpp::Time, geometry_msgs::msg::PoseStamped>> current_path_;
  size_t current_waypoint_index_{0};

  double getSpeedLimit() const { return speed_limit_; }
};

}  // namespace nav3d_controller