#include "nav3d_util/node_utils.hpp"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstring>
#include <sched.h>
#include <stdexcept>
#include <string>

namespace nav3d_util {

std::string sanitize_node_name(const std::string& potential_node_name) {
    std::string node_name = potential_node_name;

    std::replace_if(
        node_name.begin(), node_name.end(), [](unsigned char c) { return std::isalnum(c) == 0; }, '_');

    return node_name;
}

std::string add_namespaces(const std::string& top_ns, const std::string& sub_ns) {
    std::string t = top_ns;
    std::string s = sub_ns;

    if (!t.empty() && t.front() != '/') {
        t.insert(t.begin(), '/');
    }

    while (t.size() > 1 && t.back() == '/') {
        t.pop_back();
    }

    while (!s.empty() && s.front() == '/') {
        s.erase(s.begin());
    }

    if (s.empty()) {
        return t.empty() ? std::string("/") : t;
    }

    if (t.empty()) {
        return "/" + s;
    }

    return t + "/" + s;
}

std::string time_to_string(std::size_t len) {
    std::string output(len, '0');

    const auto now = std::chrono::high_resolution_clock::now();
    const auto ticks = now.time_since_epoch().count();
    const std::string ts = std::to_string(ticks);

    if (ts.size() >= len) {
        output.assign(ts.substr(ts.size() - len, len));
    } else {
        output.replace(len - ts.size(), ts.size(), ts);
    }

    return output;
}

std::string generate_internal_node_name(const std::string& prefix) {
    return sanitize_node_name(prefix) + "_" + time_to_string(8);
}

rclcpp::Node::SharedPtr generate_internal_node(const std::string& prefix) {
    auto options =
        rclcpp::NodeOptions{}.start_parameter_services(false).start_parameter_event_publisher(false).arguments(
            {"--ros-args", "-r", "__node:=" + generate_internal_node_name(prefix), "--"});

    return rclcpp::Node::make_shared("_", options);
}

void setSoftRealTimePriority() {
    sched_param sch{};
    sch.sched_priority = 49;

    if (sched_setscheduler(0, SCHED_FIFO, &sch) == -1) {
        const std::string errmsg = "Cannot set as real-time thread. Users must set: <username> hard rtprio 99 and "
                                   "<username> soft rtprio 99 in /etc/security/limits.conf to enable "
                                   "realtime prioritization! Error: ";
        throw std::runtime_error(errmsg + std::strerror(errno));
    }
}

} // namespace nav3d_util
