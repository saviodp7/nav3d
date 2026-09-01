#pragma once

#include "nav3d_util/node_thread.hpp"
#include "nav3d_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>

namespace nav3d_util {

template <typename ActionT> class SimpleActionServer {
  public:
    using ExecuteCallback = std::function<void()>;
    using CompletionCallback = std::function<void()>;

    template <typename NodeT>
    explicit SimpleActionServer(NodeT node, const std::string& action_name, ExecuteCallback execute_callback,
                                CompletionCallback completion_callback = nullptr,
                                std::chrono::milliseconds server_timeout = std::chrono::milliseconds(500),
                                bool spin_thread = false,
                                const rcl_action_server_options_t& options = rcl_action_server_get_default_options(),
                                const bool realtime = false)
        : SimpleActionServer(node->get_node_base_interface(), node->get_node_clock_interface(),
                             node->get_node_logging_interface(), node->get_node_waitables_interface(), action_name,
                             std::move(execute_callback), std::move(completion_callback), server_timeout, spin_thread,
                             options, realtime) {}

    explicit SimpleActionServer(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
                                rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface,
                                rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
                                rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface,
                                const std::string& action_name, ExecuteCallback execute_callback,
                                CompletionCallback completion_callback = nullptr,
                                std::chrono::milliseconds server_timeout = std::chrono::milliseconds(500),
                                bool spin_thread = false,
                                const rcl_action_server_options_t& options = rcl_action_server_get_default_options(),
                                const bool realtime = false)
        : node_base_interface_(std::move(node_base_interface)), node_clock_interface_(std::move(node_clock_interface)),
          node_logging_interface_(std::move(node_logging_interface)),
          node_waitables_interface_(std::move(node_waitables_interface)), action_name_(action_name),
          execute_callback_(std::move(execute_callback)), completion_callback_(std::move(completion_callback)),
          server_timeout_(server_timeout), spin_thread_(spin_thread), use_realtime_prioritization_(realtime) {
        using namespace std::placeholders;

        if (spin_thread_) {
            callback_group_ =
                node_base_interface_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        }

        action_server_ = rclcpp_action::create_server<ActionT>(
            node_base_interface_, node_clock_interface_, node_logging_interface_, node_waitables_interface_,
            action_name_, std::bind(&SimpleActionServer::handle_goal, this, _1, _2),
            std::bind(&SimpleActionServer::handle_cancel, this, _1),
            std::bind(&SimpleActionServer::handle_accepted, this, _1), options, callback_group_);

        if (spin_thread_) {
            executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
            executor_->add_callback_group(callback_group_, node_base_interface_);
            executor_thread_ = std::make_unique<nav3d_util::NodeThread>(executor_);
        }
    }

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&,
                                            std::shared_ptr<const typename ActionT::Goal>) {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);

        if (!server_active_) {
            RCLCPP_INFO(node_logging_interface_->get_logger(), "Action server is inactive. Rejecting the goal.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        debug_msg("Received request for goal acceptance");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse
    handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> handle) {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);

        if (!handle->is_active()) {
            warn_msg("Received request for goal cancellation, but the handle is inactive, rejecting.");
            return rclcpp_action::CancelResponse::REJECT;
        }

        debug_msg("Received request for goal cancellation");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void activate() {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        server_active_ = true;
        stop_execution_ = false;
    }

    void deactivate() {
        debug_msg("Deactivating...");

        {
            std::lock_guard<std::recursive_mutex> lock(update_mutex_);
            server_active_ = false;
            stop_execution_ = true;
        }

        if (!execution_future_.valid()) {
            return;
        }

        if (is_running()) {
            warn_msg("Deactivating while a goal is still executing.");
        }

        using namespace std::chrono;
        auto start_time = steady_clock::now();
        while (execution_future_.wait_for(milliseconds(100)) != std::future_status::ready) {
            info_msg("Waiting for async process to finish.");
            if (steady_clock::now() - start_time >= server_timeout_) {
                terminate_all();
                if (completion_callback_) {
                    completion_callback_();
                }
                error_msg("Action callback is still running and missed deadline to stop");
            }
        }

        debug_msg("Deactivation completed.");
    }

    bool is_running() {
        return execution_future_.valid() &&
               execution_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::timeout;
    }

    bool is_server_active() {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        return server_active_;
    }

    bool is_preempt_requested() const {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        return preempt_requested_;
    }

    bool is_cancel_requested() const {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);

        if (!current_handle_) {
            error_msg("Checking for cancel but current goal is not available");
            return false;
        }

        if (pending_handle_) {
            return pending_handle_->is_canceling();
        }

        return current_handle_->is_canceling();
    }

    const std::shared_ptr<const typename ActionT::Goal> get_current_goal() const {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);

        if (!is_active(current_handle_)) {
            error_msg("A goal is not available or has reached a final state");
            return {};
        }

        return current_handle_->get_goal();
    }

    rclcpp_action::GoalUUID get_current_goal_id() const {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);

        if (!is_active(current_handle_)) {
            error_msg("A goal is not available or has reached a final state");
            return {};
        }

        return current_handle_->get_goal_id();
    }

    const std::shared_ptr<const typename ActionT::Goal> get_pending_goal() const {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);

        if (!pending_handle_ || !pending_handle_->is_active()) {
            error_msg("Attempting to get pending goal when not available");
            return {};
        }

        return pending_handle_->get_goal();
    }

    const std::shared_ptr<const typename ActionT::Goal> accept_pending_goal() {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);

        if (!pending_handle_ || !pending_handle_->is_active()) {
            error_msg("Attempting to get pending goal when not available");
            return {};
        }

        if (is_active(current_handle_) && current_handle_ != pending_handle_) {
            debug_msg("Aborting the previous goal");
            current_handle_->abort(empty_result());
        }

        current_handle_ = pending_handle_;
        pending_handle_.reset();
        preempt_requested_ = false;

        debug_msg("Preempted goal");
        return current_handle_->get_goal();
    }

    void terminate_pending_goal() {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);

        if (!pending_handle_ || !pending_handle_->is_active()) {
            error_msg("Attempting to terminate pending goal when not available");
            return;
        }

        terminate(pending_handle_);
        preempt_requested_ = false;
        debug_msg("Pending goal terminated");
    }

    void
    terminate_all(std::shared_ptr<typename ActionT::Result> result = std::make_shared<typename ActionT::Result>()) {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        terminate(current_handle_, result);
        terminate(pending_handle_, result);
        preempt_requested_ = false;
    }

    void
    terminate_current(std::shared_ptr<typename ActionT::Result> result = std::make_shared<typename ActionT::Result>()) {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        terminate(current_handle_, result);
    }

    void
    succeeded_current(std::shared_ptr<typename ActionT::Result> result = std::make_shared<typename ActionT::Result>()) {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);

        if (is_active(current_handle_)) {
            debug_msg("Setting succeed on current goal.");
            current_handle_->succeed(std::move(result));
            current_handle_.reset();
        }
    }

    void publish_feedback(std::shared_ptr<typename ActionT::Feedback> feedback) {
        if (!is_active(current_handle_)) {
            error_msg("Trying to publish feedback when the current goal handle is not active");
            return;
        }

        current_handle_->publish_feedback(std::move(feedback));
    }

  protected:
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface_;
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface_;
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface_;
    rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface_;
    std::string action_name_;

    ExecuteCallback execute_callback_;
    CompletionCallback completion_callback_;

    std::future<void> execution_future_;
    bool stop_execution_{false};
    bool use_realtime_prioritization_{false};

    mutable std::recursive_mutex update_mutex_;
    bool server_active_{false};
    bool preempt_requested_{false};
    std::chrono::milliseconds server_timeout_;

    std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> current_handle_;
    std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> pending_handle_;

    typename rclcpp_action::Server<ActionT>::SharedPtr action_server_;
    bool spin_thread_{false};

    rclcpp::CallbackGroup::SharedPtr callback_group_{nullptr};
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    std::unique_ptr<nav3d_util::NodeThread> executor_thread_;

    std::shared_ptr<typename ActionT::Result> empty_result() const {
        return std::make_shared<typename ActionT::Result>();
    }

    bool is_active(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>& handle) const {
        return handle && handle->is_active();
    }

    template <typename T, typename = void> struct has_error_msg : std::false_type {};
    template <typename T> struct has_error_msg<T, std::void_t<decltype(T::error_msg)>> : std::true_type {};

    template <typename T, typename = void> struct has_error_code : std::false_type {};
    template <typename T> struct has_error_code<T, std::void_t<decltype(T::error_code)>> : std::true_type {};

    void log_error_details_if_available(const std::shared_ptr<typename ActionT::Result>& result) {
        if constexpr (has_error_code<typename ActionT::Result>::value &&
                      has_error_msg<typename ActionT::Result>::value) {
            warn_msg("Aborting handle. error_code:" + std::to_string(result->error_code) + ", error_msg:'" +
                     result->error_msg + "'.");
        } else if constexpr (has_error_code<typename ActionT::Result>::value) {
            warn_msg("Aborting handle. error_code:" + std::to_string(result->error_code) + ".");
        } else {
            warn_msg("Aborting handle.");
        }
    }

    void terminate(std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>& handle,
                   std::shared_ptr<typename ActionT::Result> result = std::make_shared<typename ActionT::Result>()) {
        if (!is_active(handle)) {
            return;
        }

        if (handle->is_canceling()) {
            info_msg("Client requested to cancel the goal. Cancelling.");
            handle->canceled(std::move(result));
        } else {
            log_error_details_if_available(result);
            handle->abort(std::move(result));
        }
        handle.reset();
    }

    void setSoftRealTimePriority() {
        if (use_realtime_prioritization_) {
            nav3d_util::setSoftRealTimePriority();
            debug_msg("Soft realtime prioritization successfully set!");
        }
    }

    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> handle) {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        debug_msg("Receiving a new goal");

        if (is_active(current_handle_) || is_running()) {
            debug_msg("An older goal is active, moving the new goal to a pending slot.");

            if (is_active(pending_handle_)) {
                debug_msg("The pending slot is occupied. The previous pending goal will be aborted and replaced.");
                terminate(pending_handle_);
            }
            pending_handle_ = handle;
            preempt_requested_ = true;
            return;
        }

        if (is_active(pending_handle_)) {
            error_msg("Forgot to handle a preemption. Terminating the pending goal.");
            terminate(pending_handle_);
            preempt_requested_ = false;
        }

        current_handle_ = handle;

        debug_msg("Executing goal asynchronously.");
        execution_future_ = std::async(std::launch::async, [this]() {
            setSoftRealTimePriority();
            work();
        });
    }

    void work() {
        while (rclcpp::ok() && !stop_execution_ && is_active(current_handle_)) {
            debug_msg("Executing the goal...");
            try {
                execute_callback_();
            } catch (const std::exception& ex) {
                RCLCPP_ERROR(node_logging_interface_->get_logger(),
                             "Action server failed while executing action callback: \"%s\"", ex.what());
                terminate_all();
                if (completion_callback_) {
                    completion_callback_();
                }
                return;
            }

            debug_msg("Blocking processing of new goal handles.");
            std::lock_guard<std::recursive_mutex> lock(update_mutex_);

            if (stop_execution_) {
                warn_msg("Stopping the thread per request.");
                terminate_all();
                if (completion_callback_) {
                    completion_callback_();
                }
                break;
            }

            if (is_active(current_handle_)) {
                warn_msg("Current goal was not completed successfully.");
                terminate(current_handle_);
                if (completion_callback_) {
                    completion_callback_();
                }
            }

            if (is_active(pending_handle_)) {
                debug_msg("Executing a pending handle on the existing thread.");
                accept_pending_goal();
            } else {
                debug_msg("Done processing available goals.");
                break;
            }
        }

        debug_msg("Worker thread done.");
    }

    void info_msg(const std::string& msg) const {
        RCLCPP_INFO(node_logging_interface_->get_logger(), "[%s] [ActionServer] %s", action_name_.c_str(), msg.c_str());
    }

    void debug_msg(const std::string& msg) const {
        RCLCPP_DEBUG(node_logging_interface_->get_logger(), "[%s] [ActionServer] %s", action_name_.c_str(),
                     msg.c_str());
    }

    void error_msg(const std::string& msg) const {
        RCLCPP_ERROR(node_logging_interface_->get_logger(), "[%s] [ActionServer] %s", action_name_.c_str(),
                     msg.c_str());
    }

    void warn_msg(const std::string& msg) const {
        RCLCPP_WARN(node_logging_interface_->get_logger(), "[%s] [ActionServer] %s", action_name_.c_str(), msg.c_str());
    }
};

} // namespace nav3d_util
