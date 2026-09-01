#pragma once

#include "nav3d_behavior_tree/bt_action_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace nav3d_util {
class OdomSmoother;
}

namespace nav3d_core {

struct FeedbackUtils {
    std::string robot_frame;
    std::string global_frame;
    double transform_tolerance;
    std::shared_ptr<tf2_ros::Buffer> tf;
};

class NavigatorMuxer {
  public:
    NavigatorMuxer() : current_navigator_(std::string("")) {}

    bool isNavigating() {
        std::scoped_lock l(mutex_);
        return !current_navigator_.empty();
    }

    void startNavigating(const std::string& navigator_name) {
        std::scoped_lock l(mutex_);
        if (!current_navigator_.empty()) {
            RCLCPP_ERROR(rclcpp::get_logger("NavigatorMutex"),
                         "Major error! Navigation requested while another navigation"
                         " task is in progress! This likely occurred from an incorrect"
                         "implementation of a navigator plugin.");
        }
        current_navigator_ = navigator_name;
    }

    void stopNavigating(const std::string& navigator_name) {
        std::scoped_lock l(mutex_);
        if (current_navigator_ != navigator_name) {
            RCLCPP_ERROR(rclcpp::get_logger("NavigatorMutex"),
                         "Major error! Navigation stopped while another navigation"
                         " task is in progress! This likely occurred from an incorrect"
                         "implementation of a navigator plugin.");
        } else {
            current_navigator_ = std::string("");
        }
    }

  protected:
    std::string current_navigator_;
    std::mutex mutex_;
};

class NavigatorBase {
  public:
    NavigatorBase() = default;
    virtual ~NavigatorBase() = default;

    virtual bool on_configure(rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
                              const std::vector<std::string>& plugin_lib_names, const FeedbackUtils& feedback_utils,
                              nav3d_core::NavigatorMuxer* plugin_muxer,
                              std::shared_ptr<nav3d_util::OdomSmoother> odom_smoother) = 0;

    virtual bool on_activate() = 0;
    virtual bool on_deactivate() = 0;
    virtual bool on_cleanup() = 0;
};

template <class ActionT> class BehaviorTreeNavigator : public NavigatorBase {
  public:
    using Ptr = std::shared_ptr<nav3d_core::BehaviorTreeNavigator<ActionT>>;

    BehaviorTreeNavigator() : NavigatorBase() { plugin_muxer_ = nullptr; }

    virtual ~BehaviorTreeNavigator() = default;

    bool on_configure(rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
                      const std::vector<std::string>& plugin_lib_names, const FeedbackUtils& feedback_utils,
                      nav3d_core::NavigatorMuxer* plugin_muxer,
                      std::shared_ptr<nav3d_util::OdomSmoother> odom_smoother) final {
        auto node = parent_node.lock();
        logger_ = node->get_logger();
        clock_ = node->get_clock();
        feedback_utils_ = feedback_utils;
        plugin_muxer_ = plugin_muxer;

        std::string default_bt_xml_filename = getDefaultBTFilepath(parent_node);

        bt_action_server_ = std::make_unique<nav3d_behavior_tree::BtActionServer<ActionT>>(
            node, getName(), plugin_lib_names, default_bt_xml_filename,
            std::bind(&BehaviorTreeNavigator::onGoalReceived, this, std::placeholders::_1),
            std::bind(&BehaviorTreeNavigator::onLoop, this),
            std::bind(&BehaviorTreeNavigator::onPreempt, this, std::placeholders::_1),
            std::bind(&BehaviorTreeNavigator::onCompletion, this, std::placeholders::_1, std::placeholders::_2));

        bool ok = true;
        if (!bt_action_server_->on_configure()) {
            ok = false;
        }

        BT::Blackboard::Ptr blackboard = bt_action_server_->getBlackboard();
        blackboard->set("tf_buffer", feedback_utils.tf);
        blackboard->set("initial_pose_received", false);
        blackboard->set("number_recoveries", 0);
        blackboard->set("odom_smoother", odom_smoother);

        return configure(parent_node, odom_smoother) && ok;
    }

    bool on_activate() final {
        bool ok = true;
        if (!bt_action_server_->on_activate()) {
            ok = false;
        }
        return activate() && ok;
    }

    bool on_deactivate() final {
        bool ok = true;
        if (!bt_action_server_->on_deactivate()) {
            ok = false;
        }
        return deactivate() && ok;
    }

    bool on_cleanup() final {
        bool ok = true;
        if (!bt_action_server_->on_cleanup()) {
            ok = false;
        }

        bt_action_server_.reset();

        return cleanup() && ok;
    }

    virtual std::string getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr node) = 0;
    virtual std::string getName() = 0;

  protected:
    bool onGoalReceived(typename ActionT::Goal::ConstSharedPtr goal) {
        if (plugin_muxer_->isNavigating()) {
            RCLCPP_ERROR(logger_,
                         "Requested navigation from %s while another navigator is processing,"
                         " rejecting request.",
                         getName().c_str());
            return false;
        }

        bool goal_accepted = goalReceived(goal);

        if (goal_accepted) {
            plugin_muxer_->startNavigating(getName());
        }

        return goal_accepted;
    }

    void onCompletion(typename ActionT::Result::SharedPtr result, const nav3d_behavior_tree::BtStatus final_bt_status) {
        plugin_muxer_->stopNavigating(getName());
        goalCompleted(result, final_bt_status);
    }

    virtual bool goalReceived(typename ActionT::Goal::ConstSharedPtr goal) = 0;
    virtual void onLoop() = 0;
    virtual void onPreempt(typename ActionT::Goal::ConstSharedPtr goal) = 0;

    virtual void goalCompleted(typename ActionT::Result::SharedPtr result,
                               const nav3d_behavior_tree::BtStatus final_bt_status) = 0;

    virtual bool configure(rclcpp_lifecycle::LifecycleNode::WeakPtr, std::shared_ptr<nav3d_util::OdomSmoother>) {
        return true;
    }

    virtual bool cleanup() { return true; }
    virtual bool activate() { return true; }
    virtual bool deactivate() { return true; }

    std::unique_ptr<nav3d_behavior_tree::BtActionServer<ActionT>> bt_action_server_;
    rclcpp::Logger logger_{rclcpp::get_logger("Navigator")};
    rclcpp::Clock::SharedPtr clock_;
    FeedbackUtils feedback_utils_;
    NavigatorMuxer* plugin_muxer_;
};

} // namespace nav3d_core
