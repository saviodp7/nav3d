#pragma once

#include "angles/angles.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav3d_core/controller.hpp"
#include "nav3d_msgs/action/follow_path.hpp"
#include "nav3d_msgs/msg/speed_limit.hpp"
#include "nav3d_msgs/msg/trajectory_point.hpp"
#include "nav3d_util/lifecycle_node.hpp"
#include "nav3d_util/robot_utils.hpp"
#include "nav3d_util/simple_action_server.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
#include "tf2/utils.h"
#include "tf2_ros/transform_listener.h"

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace nav3d_controller {

class ControllerServer : public nav3d_util::LifecycleNode {
  public:
    using ControllerMap = std::unordered_map<std::string, nav3d_core::Controller::Ptr>;
    using Action = nav3d_msgs::action::FollowPath;
    using ActionServer = nav3d_util::SimpleActionServer<Action>;

    explicit ControllerServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~ControllerServer();

  protected:
    nav3d_util::CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
    nav3d_util::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
    nav3d_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
    nav3d_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
    nav3d_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

    std::unique_ptr<ActionServer> action_server_;

    rclcpp::Publisher<nav3d_msgs::msg::TrajectoryPoint>::SharedPtr waypoint_pub_;

    void computeControl();

    void generateTimedTrajectory(const nav_msgs::msg::Path& path);
    std::optional<nav3d_msgs::msg::TrajectoryPoint> getCurrentCommand();
    bool isGoalReached();

    rclcpp::Subscription<nav3d_msgs::msg::SpeedLimit>::SharedPtr speed_limit_sub_;
    void speedLimitCallback(const nav3d_msgs::msg::SpeedLimit::SharedPtr msg);

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::string global_frame_{"map"};
    std::string robot_base_frame_{"base_link"};
    double transform_tolerance_{0.1};

    std::string waypoint_topic_{"offboard/waypoint"};
    double controller_frequency_{50.0};

    double xyz_goal_tolerance_{0.1};
    double yaw_goal_tolerance_{0.1};

    double speed_limit_{1.0};
    double default_speed_limit_{0.3};

    double slowdown_distance_{0.3};
    double min_speed_{0.0};

    rclcpp::Time last_valid_cmd_time_;

    rclcpp::Time path_start_time_{0, 0, RCL_ROS_TIME};
    std::vector<std::pair<rclcpp::Time, geometry_msgs::msg::PoseStamped>> current_path_;
    size_t current_waypoint_index_{0};

    double getSpeedLimit() const { return speed_limit_; }
};

} // namespace nav3d_controller
