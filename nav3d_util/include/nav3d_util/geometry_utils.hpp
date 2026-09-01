#pragma once

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace nav3d_util::geometry_utils {
[[nodiscard]] inline geometry_msgs::msg::Quaternion orientationAroundZAxis(double angle) noexcept {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, angle);
    return tf2::toMsg(q);
}

[[nodiscard]] inline double euclidean_distance(const geometry_msgs::msg::Point& pos1,
                                               const geometry_msgs::msg::Point& pos2,
                                               const bool is_3d = true) noexcept {
    const double dx = pos1.x - pos2.x;
    const double dy = pos1.y - pos2.y;

    if (is_3d) {
        const double dz = pos1.z - pos2.z;
        return std::hypot(dx, dy, dz);
    }

    return std::hypot(dx, dy);
}

[[nodiscard]] inline double euclidean_distance(const geometry_msgs::msg::Pose& pos1,
                                               const geometry_msgs::msg::Pose& pos2, const bool is_3d = true) noexcept {
    const double dx = pos1.position.x - pos2.position.x;
    const double dy = pos1.position.y - pos2.position.y;

    if (is_3d) {
        const double dz = pos1.position.z - pos2.position.z;
        return std::hypot(dx, dy, dz);
    }

    return std::hypot(dx, dy);
}

[[nodiscard]] inline double euclidean_distance(const geometry_msgs::msg::PoseStamped& pos1,
                                               const geometry_msgs::msg::PoseStamped& pos2,
                                               const bool is_3d = true) noexcept {
    return euclidean_distance(pos1.pose, pos2.pose, is_3d);
}

[[nodiscard]] inline double euclidean_distance(const geometry_msgs::msg::Pose2D& pos1,
                                               const geometry_msgs::msg::Pose2D& pos2) noexcept {
    const double dx = pos1.x - pos2.x;
    const double dy = pos1.y - pos2.y;
    return std::hypot(dx, dy);
}

template <typename Iter, typename Getter> inline Iter min_by(Iter begin, Iter end, Getter getCompareVal) {
    if (begin == end) {
        return end;
    }

    auto lowest = getCompareVal(*begin);
    Iter lowest_it = begin;

    for (Iter it = ++begin; it != end; ++it) {
        auto comp = getCompareVal(*it);
        if (comp <= lowest) {
            lowest = comp;
            lowest_it = it;
        }
    }

    return lowest_it;
}

template <typename Iter, typename T>
inline Iter first_after_integrated_distance(Iter begin, Iter end, const T& distance_threshold) {
    if (begin == end) {
        return end;
    }

    double dist = 0.0;
    for (Iter it = begin; it != end - 1; ++it) {
        dist += euclidean_distance(*it, *(it + 1));
        if (dist > distance_threshold) {
            return it + 1;
        }
    }

    return end;
}

[[nodiscard]] inline double calculate_path_length(const nav_msgs::msg::Path& path,
                                                  std::size_t start_index = 0) noexcept {
    if (start_index + 1 >= path.poses.size()) {
        return 0.0;
    }

    double path_length = 0.0;
    for (std::size_t idx = start_index; idx < path.poses.size() - 1; ++idx) {
        path_length += euclidean_distance(path.poses[idx].pose, path.poses[idx + 1].pose, true);
    }

    return path_length;
}

} // namespace nav3d_util::geometry_utils
