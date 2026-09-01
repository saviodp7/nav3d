#pragma once

#include "nav3d_util/string_utils.hpp"

#include <chrono>
#include <string>
#include <vector>

namespace nav3d_util {

void startup_lifecycle_nodes(const std::vector<std::string>& node_names,
                             const std::chrono::seconds service_call_timeout = std::chrono::seconds::max(),
                             int retries = 3);

inline void startup_lifecycle_nodes(const std::string& nodes,
                                    const std::chrono::seconds service_call_timeout = std::chrono::seconds::max(),
                                    int retries = 3) {
    startup_lifecycle_nodes(nav3d_util::split(nodes, ':'), service_call_timeout, retries);
}

void reset_lifecycle_nodes(const std::vector<std::string>& node_names,
                           const std::chrono::seconds service_call_timeout = std::chrono::seconds::max(),
                           int retries = 3);

inline void reset_lifecycle_nodes(const std::string& nodes,
                                  const std::chrono::seconds service_call_timeout = std::chrono::seconds::max(),
                                  int retries = 3) {
    reset_lifecycle_nodes(nav3d_util::split(nodes, ':'), service_call_timeout, retries);
}

} // namespace nav3d_util
