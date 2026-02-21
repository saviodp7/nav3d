#pragma once

#include <memory>
#include <string>
#include <vector>

#include "nav3d_util/lifecycle_node.hpp"
#include "nav3d_util/odometry_utils.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "nav3d_core/behavior_tree_navigator.hpp"
#include "pluginlib/class_loader.hpp"

namespace nav3d_bt_navigator
{

class BtNavigator : public nav3d_util::LifecycleNode
{
public:
  explicit BtNavigator(rclcpp::NodeOptions options = rclcpp::NodeOptions());
  ~BtNavigator();

protected:
  nav3d_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  nav3d_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  nav3d_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  nav3d_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  nav3d_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  pluginlib::ClassLoader<nav3d_core::NavigatorBase> class_loader_;
  std::vector<pluginlib::UniquePtr<nav3d_core::NavigatorBase>> navigators_;
  nav3d_core::NavigatorMuxer plugin_muxer_;

  std::shared_ptr<nav3d_util::OdomSmoother> odom_smoother_;

  std::string robot_frame_;
  std::string global_frame_;
  double transform_tolerance_;
  double filter_duration_;
  std::string odom_topic_;

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace nav3d_bt_navigator