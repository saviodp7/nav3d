#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "nav3d_behavior_tree/behavior_tree_engine.hpp"
#include "nav3d_behavior_tree/ros_topic_logger.hpp"
#include "nav3d_util/lifecycle_node.hpp"
#include "nav3d_util/simple_action_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace nav3d_behavior_tree
{

template<class ActionT>
class BtActionServer
{
public:
  using ActionServer = nav3d_util::SimpleActionServer<ActionT>;

  using OnGoalReceivedCallback = std::function<bool (typename ActionT::Goal::ConstSharedPtr)>;
  using OnLoopCallback = std::function<void ()>;
  using OnPreemptCallback = std::function<void (typename ActionT::Goal::ConstSharedPtr)>;
  using OnCompletionCallback = std::function<void (typename ActionT::Result::SharedPtr, nav3d_behavior_tree::BtStatus)>;

  explicit BtActionServer(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & action_name,
    const std::vector<std::string> & plugin_lib_names,
    const std::string & default_bt_xml_filename,
    OnGoalReceivedCallback on_goal_received_callback,
    OnLoopCallback on_loop_callback,
    OnPreemptCallback on_preempt_callback,
    OnCompletionCallback on_completion_callback);

  ~BtActionServer();

  bool on_configure();
  bool on_activate();
  bool on_deactivate();
  bool on_cleanup();

  void setGrootMonitoring(const bool enable, const unsigned server_port);

  bool loadBehaviorTree(const std::string & bt_xml_filename = "");

  BT::Blackboard::Ptr getBlackboard() const
  {
    return blackboard_;
  }

  std::string getCurrentBTFilename() const
  {
    return current_bt_xml_filename_;
  }

  std::string getDefaultBTFilename() const
  {
    return default_bt_xml_filename_;
  }

  const std::shared_ptr<const typename ActionT::Goal> acceptPendingGoal()
  {
    return action_server_->accept_pending_goal();
  }

  void terminatePendingGoal()
  {
    action_server_->terminate_pending_goal();
  }

  const std::shared_ptr<const typename ActionT::Goal> getCurrentGoal() const
  {
    return action_server_->get_current_goal();
  }

  const std::shared_ptr<const typename ActionT::Goal> getPendingGoal() const
  {
    return action_server_->get_pending_goal();
  }

  void publishFeedback(typename std::shared_ptr<typename ActionT::Feedback> feedback)
  {
    action_server_->publish_feedback(feedback);
  }

  const BT::Tree & getTree() const
  {
    return tree_;
  }

  void haltTree()
  {
    tree_.haltTree();
  }

  void setInternalError(uint16_t error_code, const std::string & error_msg);
  void resetInternalError(void);
  bool populateInternalError(typename std::shared_ptr<typename ActionT::Result> result);

protected:
  void executeCallback();
  void populateErrorCode(typename std::shared_ptr<typename ActionT::Result> result);
  void cleanErrorCodes();

  std::string action_name_;
  std::shared_ptr<ActionServer> action_server_;
  BT::Tree tree_;
  BT::Blackboard::Ptr blackboard_;

  std::string current_bt_xml_filename_;
  std::string default_bt_xml_filename_;

  std::unique_ptr<nav3d_behavior_tree::BehaviorTreeEngine> bt_;
  std::vector<std::string> plugin_lib_names_;
  std::vector<std::string> error_code_name_prefixes_;

  rclcpp::Node::SharedPtr client_node_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("BtActionServer")};

  std::unique_ptr<RosTopicLogger> topic_logger_;

  std::chrono::milliseconds bt_loop_duration_;
  std::chrono::milliseconds default_server_timeout_;
  std::chrono::milliseconds wait_for_service_timeout_;

  bool always_reload_bt_xml_ = false;

  bool enable_groot_monitoring_ = true;
  int groot_server_port_ = 1667;

  OnGoalReceivedCallback on_goal_received_callback_;
  OnLoopCallback on_loop_callback_;
  OnPreemptCallback on_preempt_callback_;
  OnCompletionCallback on_completion_callback_;

  uint16_t internal_error_code_;
  std::string internal_error_msg_;
};

}  // namespace nav3d_behavior_tree

#include "nav3d_behavior_tree/bt_action_server_impl.hpp"