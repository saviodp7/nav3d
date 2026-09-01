#include "lifecycle_msgs/msg/transition.hpp"
#include "nav3d_util/lifecycle_service_client.hpp"

#include <chrono>
#include <stdexcept>
#include <string>
#include <vector>

namespace nav3d_util {

template <typename Fn> static void retry_runtime_error(Fn&& fn, int retries) {
    if (retries < 0) {
        retries = 0;
    }

    int attempts = 0;
    while (true) {
        try {
            fn();
            return;
        } catch (const std::runtime_error&) {
            ++attempts;
            if (attempts > retries) {
                throw;
            }
        }
    }
}

static void startupLifecycleNode(const std::string& node_name, const std::chrono::seconds service_call_timeout,
                                 const int retries) {
    LifecycleServiceClient sc(node_name);

    retry_runtime_error(
        [&]() { sc.change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, service_call_timeout); },
        retries);

    retry_runtime_error(
        [&]() { sc.change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, service_call_timeout); },
        retries);
}

void startup_lifecycle_nodes(const std::vector<std::string>& node_names,
                             const std::chrono::seconds service_call_timeout, const int retries) {
    for (const auto& node_name : node_names) {
        startupLifecycleNode(node_name, service_call_timeout, retries);
    }
}

static void resetLifecycleNode(const std::string& node_name, const std::chrono::seconds service_call_timeout,
                               const int retries) {
    LifecycleServiceClient sc(node_name);

    retry_runtime_error(
        [&]() { sc.change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, service_call_timeout); },
        retries);

    retry_runtime_error(
        [&]() { sc.change_state(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP, service_call_timeout); }, retries);
}

void reset_lifecycle_nodes(const std::vector<std::string>& node_names, const std::chrono::seconds service_call_timeout,
                           const int retries) {
    for (const auto& node_name : node_names) {
        resetLifecycleNode(node_name, service_call_timeout, retries);
    }
}

} // namespace nav3d_util
