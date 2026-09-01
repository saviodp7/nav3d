#pragma once

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/header.hpp"

#include <array>
#include <cmath>
#include <cstddef>

namespace nav3d_util {

inline bool validateMsg(const double& num) {
    if (std::isinf(num)) {
        return false;
    }
    if (std::isnan(num)) {
        return false;
    }
    return true;
}

template <std::size_t N> inline bool validateMsg(const std::array<double, N>& msg) {
    for (const auto& element : msg) {
        if (!validateMsg(element)) {
            return false;
        }
    }
    return true;
}

inline bool validateMsg(const builtin_interfaces::msg::Time& msg) {
    constexpr int32_t NSEC_PER_SEC = 1000000000;
    if (msg.nanosec >= static_cast<uint32_t>(NSEC_PER_SEC)) {
        return false;
    }
    return true;
}

inline bool validateMsg(const std_msgs::msg::Header& msg) {
    if (!validateMsg(msg.stamp)) {
        return false;
    }
    if (msg.frame_id.empty()) {
        return false;
    }
    return true;
}

inline bool validateMsg(const geometry_msgs::msg::Point& msg) {
    if (!validateMsg(msg.x)) {
        return false;
    }
    if (!validateMsg(msg.y)) {
        return false;
    }
    if (!validateMsg(msg.z)) {
        return false;
    }
    return true;
}

inline bool validateMsg(const geometry_msgs::msg::Quaternion& msg) {
    if (!validateMsg(msg.x)) {
        return false;
    }
    if (!validateMsg(msg.y)) {
        return false;
    }
    if (!validateMsg(msg.z)) {
        return false;
    }
    if (!validateMsg(msg.w)) {
        return false;
    }

    constexpr double epsilon = 1e-4;
    const double norm_err = (msg.x * msg.x + msg.y * msg.y + msg.z * msg.z + msg.w * msg.w) - 1.0;

    if (std::abs(norm_err) >= epsilon) {
        return false;
    }

    return true;
}

inline bool validateMsg(const geometry_msgs::msg::Pose& msg) {
    if (!validateMsg(msg.position)) {
        return false;
    }
    if (!validateMsg(msg.orientation)) {
        return false;
    }
    return true;
}

inline bool validateMsg(const geometry_msgs::msg::PoseWithCovariance& msg) {
    if (!validateMsg(msg.pose)) {
        return false;
    }
    if (!validateMsg(msg.covariance)) {
        return false;
    }
    return true;
}

inline bool validateMsg(const geometry_msgs::msg::PoseWithCovarianceStamped& msg) {
    if (!validateMsg(msg.header)) {
        return false;
    }
    if (!validateMsg(msg.pose)) {
        return false;
    }
    return true;
}

inline bool validateMsg(const nav_msgs::msg::MapMetaData& msg) {
    if (!validateMsg(msg.origin)) {
        return false;
    }
    if (!validateMsg(msg.resolution)) {
        return false;
    }

    if (msg.height == 0 || msg.width == 0) {
        return false;
    }
    return true;
}

inline bool validateMsg(const nav_msgs::msg::OccupancyGrid& msg) {
    if (!validateMsg(msg.header)) {
        return false;
    }
    if (!validateMsg(msg.info)) {
        return false;
    }

    if (msg.data.size() != static_cast<std::size_t>(msg.info.width) * static_cast<std::size_t>(msg.info.height)) {
        return false;
    }

    return true;
}

} // namespace nav3d_util
