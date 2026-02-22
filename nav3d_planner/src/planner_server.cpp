#include "nav3d_planner/planner_server.hpp"

#include <cmath>
#include <utility>

#include "builtin_interfaces/msg/duration.hpp"
#include "nav3d_util/node_utils.hpp"
#include "nav3d_util/robot_utils.hpp"
#include "rcl/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/create_timer_ros.h"

using namespace std::chrono_literals;
using nav3d_util::declare_parameter_if_not_declared;

namespace
{

rclcpp::NodeOptions withAutoDeclaredParams(const rclcpp::NodeOptions & in)
{
  auto out = in;
  out.automatically_declare_parameters_from_overrides(true);
  return out;
}

builtin_interfaces::msg::Duration toBuiltinDuration(std::chrono::nanoseconds ns)
{
  builtin_interfaces::msg::Duration out;
  const auto sec = std::chrono::duration_cast<std::chrono::seconds>(ns);
  const auto rem = ns - sec;

  out.sec = static_cast<int32_t>(sec.count());
  out.nanosec = static_cast<uint32_t>(rem.count());
  return out;
}

}  // namespace

namespace nav3d_planner
{

PlannerServer::PlannerServer(const rclcpp::NodeOptions & options)
: nav3d_util::LifecycleNode("planner_server", "", withAutoDeclaredParams(options)),
  planner_loader_("nav3d_core", "nav3d_core::GlobalPlanner")
{
  RCLCPP_INFO(get_logger(), "Creating planner_server");

  declare_parameter_if_not_declared(
    this, "expected_planner_frequency", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    this, "action_server_result_timeout", rclcpp::ParameterValue(10.0));

  declare_parameter_if_not_declared(
    this, "global_frame", rclcpp::ParameterValue(std::string("map")));
  declare_parameter_if_not_declared(
    this, "robot_base_frame", rclcpp::ParameterValue(std::string("base_link")));
  declare_parameter_if_not_declared(
    this, "transform_tolerance", rclcpp::ParameterValue(0.1));

  declare_parameter_if_not_declared(
    this, "planner_plugins", rclcpp::ParameterValue(std::vector<std::string>{"GridBased"}));
}

nav3d_util::CallbackReturn PlannerServer::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Configuring");

  expected_planner_frequency_ = get_parameter("expected_planner_frequency").as_double();
  action_server_result_timeout_ = get_parameter("action_server_result_timeout").as_double();

  global_frame_ = get_parameter("global_frame").as_string();
  robot_base_frame_ = get_parameter("robot_base_frame").as_string();
  transform_tolerance_ = get_parameter("transform_tolerance").as_double();

  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(), get_node_timers_interface());
  tf_->setCreateTimerInterface(timer_interface);
  tf_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_, this, false);

  plan_publisher_ = create_publisher<nav_msgs::msg::Path>("plan", 1);

  planner_ids_ = get_parameter("planner_plugins").as_string_array();
  if (planner_ids_.empty()) {
    RCLCPP_FATAL(get_logger(), "Parameter 'planner_plugins' is empty");
    return nav3d_util::CallbackReturn::FAILURE;
  }

  planners_.clear();

  for (const auto & id : planner_ids_) {
    const std::string default_type = "nav3d_planner_plugin_moveit/MoveItGlobalPlanner";
    declare_parameter_if_not_declared(this, id + ".plugin", rclcpp::ParameterValue(default_type));

    try {
      const std::string type = nav3d_util::get_plugin_type_param(this, id);
      RCLCPP_INFO(get_logger(), "Creating planner id '%s' of type '%s'", id.c_str(), type.c_str());

      auto planner = planner_loader_.createSharedInstance(type);
      planner->configure(rclcpp_lifecycle::LifecycleNode::WeakPtr(shared_from_this()), id, tf_);
      planners_.emplace(id, std::move(planner));
    } catch (const std::exception & ex) {
      RCLCPP_FATAL(get_logger(), "Failed to create planner id '%s': %s", id.c_str(), ex.what());
      return nav3d_util::CallbackReturn::FAILURE;
    }
  }

  rcl_action_server_options_t server_options = rcl_action_server_get_default_options();
  server_options.result_timeout.nanoseconds = RCL_S_TO_NS(action_server_result_timeout_);

  action_server_pose_ = std::make_unique<ActionServerToPose>(
    shared_from_this(),
    "compute_path_to_pose",
    std::bind(&PlannerServer::computePlan, this),
    nullptr,
    std::chrono::milliseconds(500),
    true,
    server_options);

  return nav3d_util::CallbackReturn::SUCCESS;
}

nav3d_util::CallbackReturn PlannerServer::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Activating");

  plan_publisher_->on_activate();
  action_server_pose_->activate();

  for (auto & kv : planners_) {
    kv.second->activate();
  }

  createBond();
  return nav3d_util::CallbackReturn::SUCCESS;
}

nav3d_util::CallbackReturn PlannerServer::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Deactivating");

  action_server_pose_->deactivate();
  plan_publisher_->on_deactivate();

  for (auto & kv : planners_) {
    kv.second->deactivate();
  }

  destroyBond();
  return nav3d_util::CallbackReturn::SUCCESS;
}

nav3d_util::CallbackReturn PlannerServer::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Cleaning up");

  action_server_pose_.reset();
  plan_publisher_.reset();

  planners_.clear();
  planner_ids_.clear();

  tf_listener_.reset();
  tf_.reset();

  return nav3d_util::CallbackReturn::SUCCESS;
}

nav3d_util::CallbackReturn PlannerServer::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav3d_util::CallbackReturn::SUCCESS;
}

template<typename T>
bool PlannerServer::isServerInactive(std::unique_ptr<nav3d_util::SimpleActionServer<T>> & action_server)
{
  if (action_server == nullptr || !action_server->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
    return true;
  }
  return false;
}

template<typename T>
bool PlannerServer::isCancelRequested(std::unique_ptr<nav3d_util::SimpleActionServer<T>> & action_server)
{
  if (action_server->is_cancel_requested()) {
    RCLCPP_INFO(get_logger(), "Goal was canceled. Canceling planning action.");
    action_server->terminate_all();
    return true;
  }
  return false;
}

template<typename T>
void PlannerServer::getPreemptedGoalIfRequested(
  std::unique_ptr<nav3d_util::SimpleActionServer<T>> & action_server,
  typename std::shared_ptr<const typename T::Goal> & goal)
{
  if (action_server->is_preempt_requested()) {
    goal = action_server->accept_pending_goal();
  }
}

nav3d_core::GlobalPlanner::Ptr PlannerServer::getPlannerOrThrow(const std::string & planner_id)
{
  const std::string effective_id = planner_id.empty() ? planner_ids_.front() : planner_id;

  auto it = planners_.find(effective_id);
  if (it == planners_.end()) {
    throw nav3d_core::InvalidPlanner("Planner id '" + effective_id + "' not found");
  }

  return it->second;
}

void PlannerServer::computePlan()
{
  auto goal = action_server_pose_->get_current_goal();
  auto result = std::make_shared<ActionToPose::Result>();

  if (isServerInactive(action_server_pose_) || isCancelRequested(action_server_pose_)) {
    return;
  }

  getPreemptedGoalIfRequested(action_server_pose_, goal);

  auto cancel_checker = [this]() {
    return action_server_pose_->is_cancel_requested();
  };

  geometry_msgs::msg::PoseStamped start;
  geometry_msgs::msg::PoseStamped goal_pose = goal->goal;

  try {
    if (cancel_checker()) {
      action_server_pose_->terminate_all();
      return;
    }

    if (goal->use_start) {
      start = goal->start;
    } else {
      if (!nav3d_util::getCurrentPose(
            start, *tf_, global_frame_, robot_base_frame_, transform_tolerance_))
      {
        throw nav3d_core::PlannerTFError("Unable to get current robot pose");
      }
    }

    if (!start.header.frame_id.empty() && start.header.frame_id != global_frame_) {
      geometry_msgs::msg::PoseStamped transformed;
      if (!nav3d_util::transformPoseInTargetFrame(
            start, transformed, *tf_, global_frame_, transform_tolerance_))
      {
        throw nav3d_core::PlannerTFError("Unable to transform start pose to global frame");
      }
      start = std::move(transformed);
    }

    if (!goal_pose.header.frame_id.empty() && goal_pose.header.frame_id != global_frame_) {
      geometry_msgs::msg::PoseStamped transformed;
      if (!nav3d_util::transformPoseInTargetFrame(
            goal_pose, transformed, *tf_, global_frame_, transform_tolerance_))
      {
        throw nav3d_core::PlannerTFError("Unable to transform goal pose to global frame");
      }
      goal_pose = std::move(transformed);
    }

    if (cancel_checker()) {
      action_server_pose_->terminate_all();
      return;
    }

    auto planner = getPlannerOrThrow(goal->planner_id);

    const auto t0 = std::chrono::steady_clock::now();
    auto path = planner->createPlan(start, goal_pose, cancel_checker);
    const auto t1 = std::chrono::steady_clock::now();

    result->planning_time =
      toBuiltinDuration(std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0));

    if (cancel_checker()) {
      action_server_pose_->terminate_all();
      return;
    }

    if (path.poses.empty()) {
      throw nav3d_core::NoValidPathCouldBeFound("Planner returned an empty path");
    }

    result->path = std::move(path);
    result->error_code = ActionToPoseResult::NONE;
    result->error_msg.clear();

    publishPlan(result->path);
    action_server_pose_->succeeded_current(result);

  } catch (const nav3d_core::InvalidPlanner & ex) {
    result->error_code = ActionToPoseResult::INVALID_PLANNER;
    result->error_msg = ex.what();
    action_server_pose_->terminate_current(result);
  } catch (const nav3d_core::PlannerTFError & ex) {
    result->error_code = ActionToPoseResult::TF_ERROR;
    result->error_msg = ex.what();
    action_server_pose_->terminate_current(result);
  } catch (const nav3d_core::StartOutsideMapBounds & ex) {
    result->error_code = ActionToPoseResult::START_OUTSIDE_MAP;
    result->error_msg = ex.what();
    action_server_pose_->terminate_current(result);
  } catch (const nav3d_core::GoalOutsideMapBounds & ex) {
    result->error_code = ActionToPoseResult::GOAL_OUTSIDE_MAP;
    result->error_msg = ex.what();
    action_server_pose_->terminate_current(result);
  } catch (const nav3d_core::StartOccupied & ex) {
    result->error_code = ActionToPoseResult::START_OCCUPIED;
    result->error_msg = ex.what();
    action_server_pose_->terminate_current(result);
  } catch (const nav3d_core::GoalOccupied & ex) {
    result->error_code = ActionToPoseResult::GOAL_OCCUPIED;
    result->error_msg = ex.what();
    action_server_pose_->terminate_current(result);
  } catch (const nav3d_core::PlannerTimedOut & ex) {
    result->error_code = ActionToPoseResult::TIMEOUT;
    result->error_msg = ex.what();
    action_server_pose_->terminate_current(result);
  } catch (const nav3d_core::NoValidPathCouldBeFound & ex) {
    result->error_code = ActionToPoseResult::NO_VALID_PATH;
    result->error_msg = ex.what();
    action_server_pose_->terminate_current(result);
  } catch (const std::exception & ex) {
    result->error_code = ActionToPoseResult::UNKNOWN;
    result->error_msg = std::string("Planning exception: ") + ex.what();
    action_server_pose_->terminate_current(result);
  }
}

void PlannerServer::publishPlan(const nav_msgs::msg::Path & path)
{
  if (!plan_publisher_ || !plan_publisher_->is_activated()) {
    return;
  }

  if (plan_publisher_->get_subscription_count() == 0) {
    return;
  }

  auto msg = std::make_unique<nav_msgs::msg::Path>(path);
  plan_publisher_->publish(std::move(msg));

  RCLCPP_DEBUG(get_logger(), "Published path with %zu poses for visualization", path.poses.size());

  for (size_t i = 0; i < path.poses.size(); ++i) {
    const auto & ps = path.poses[i];
    const auto & pos = ps.pose.position;
    const auto & orient = ps.pose.orientation;

    tf2::Quaternion tf_q(orient.x, orient.y, orient.z, orient.w);
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);

    RCLCPP_DEBUG(
      get_logger(),
      "   [%3zu] pos=[%7.3f, %7.3f, %7.3f] yaw=%6.1f°",
      i, pos.x, pos.y, pos.z, yaw * 180.0 / M_PI);
  }
}

}  // namespace nav3d_planner