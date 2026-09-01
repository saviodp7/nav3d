#include "nav3d_behavior_tree/plugins/action/truncate_path_local_action.hpp"

#include "behaviortree_cpp/decorator_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav3d_util/geometry_utils.hpp"
#include "nav3d_util/robot_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/buffer.h"

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace nav3d_behavior_tree {

TruncatePathLocal::TruncatePathLocal(const std::string& name, const BT::NodeConfiguration& conf)
    : BT::ActionNodeBase(name, conf) {
    tf_buffer_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
}

BT::NodeStatus TruncatePathLocal::tick() {
    setStatus(BT::NodeStatus::RUNNING);

    double distance_forward = 0.0;
    double distance_backward = 0.0;
    geometry_msgs::msg::PoseStamped pose;
    double angular_distance_weight = 0.0;
    double max_robot_pose_search_dist = std::numeric_limits<double>::infinity();

    std::string mode = "local";
    double resample_distance = 0.0;
    int max_output_poses = 0;

    getInput("distance_forward", distance_forward);
    getInput("distance_backward", distance_backward);
    getInput("angular_distance_weight", angular_distance_weight);
    getInput("max_robot_pose_search_dist", max_robot_pose_search_dist);

    getInput("mode", mode);
    getInput("resample_distance", resample_distance);
    getInput("max_output_poses", max_output_poses);

    const bool path_pruning = std::isfinite(max_robot_pose_search_dist);
    nav_msgs::msg::Path new_path;
    getInput("input_path", new_path);
    if (!path_pruning || new_path != path_) {
        path_ = new_path;
        closest_pose_detection_begin_ = path_.poses.begin();
    }

    if (!getRobotPose(path_.header.frame_id, pose)) {
        return BT::NodeStatus::FAILURE;
    }

    if (path_.poses.empty()) {
        setOutput("output_path", path_);
        return BT::NodeStatus::SUCCESS;
    }

    auto closest_pose_detection_end = path_.poses.end();
    if (path_pruning) {
        closest_pose_detection_end = nav3d_util::geometry_utils::first_after_integrated_distance(
            closest_pose_detection_begin_, path_.poses.end(), max_robot_pose_search_dist);
    }

    auto current_pose =
        nav3d_util::geometry_utils::min_by(closest_pose_detection_begin_, closest_pose_detection_end,
                                           [&pose, angular_distance_weight](const geometry_msgs::msg::PoseStamped& ps) {
                                               return poseDistance(pose, ps, angular_distance_weight);
                                           });

    if (path_pruning) {
        closest_pose_detection_begin_ = current_pose;
    }

    auto forward_pose_it =
        nav3d_util::geometry_utils::first_after_integrated_distance(current_pose, path_.poses.end(), distance_forward);

    if (forward_pose_it == current_pose && (current_pose + 1) != path_.poses.end()) {
        forward_pose_it = current_pose + 1;
    }

    nav_msgs::msg::Path output_path;
    output_path.header = path_.header;

    if (mode == "remaining") {
        output_path.poses = std::vector<geometry_msgs::msg::PoseStamped>(current_pose, forward_pose_it);
        if (output_path.poses.empty()) {
            output_path.poses.push_back(*current_pose);
        }
    } else {
        auto backward_pose_it = nav3d_util::geometry_utils::first_after_integrated_distance(
            std::reverse_iterator(current_pose + 1), path_.poses.rend(), distance_backward);

        output_path.poses = std::vector<geometry_msgs::msg::PoseStamped>(backward_pose_it.base(), forward_pose_it);
    }

    if (resample_distance > 0.0) {
        resamplePath(output_path, resample_distance);
    }

    if (max_output_poses > 0) {
        capPath(output_path, max_output_poses);
    }

    setOutput("output_path", output_path);
    return BT::NodeStatus::SUCCESS;
}

bool TruncatePathLocal::getRobotPose(std::string path_frame_id, geometry_msgs::msg::PoseStamped& pose) {
    if (!getInput("pose", pose)) {
        std::string robot_frame;
        if (!getInput("robot_frame", robot_frame)) {
            RCLCPP_ERROR(config().blackboard->get<rclcpp::Node::SharedPtr>("node")->get_logger(),
                         "Neither pose nor robot_frame specified for %s", name().c_str());
            return false;
        }

        double transform_tolerance = 0.0;
        getInput("transform_tolerance", transform_tolerance);

        if (!nav3d_util::getCurrentPose(pose, *tf_buffer_, path_frame_id, robot_frame, transform_tolerance)) {
            RCLCPP_WARN(config().blackboard->get<rclcpp::Node::SharedPtr>("node")->get_logger(),
                        "Failed to lookup current robot pose for %s", name().c_str());
            return false;
        }
    }
    return true;
}

double TruncatePathLocal::poseDistance(const geometry_msgs::msg::PoseStamped& pose1,
                                       const geometry_msgs::msg::PoseStamped& pose2,
                                       const double angular_distance_weight) {
    const double dx = pose1.pose.position.x - pose2.pose.position.x;
    const double dy = pose1.pose.position.y - pose2.pose.position.y;
    const double dz = pose1.pose.position.z - pose2.pose.position.z;

    tf2::Quaternion q1;
    tf2::convert(pose1.pose.orientation, q1);
    tf2::Quaternion q2;
    tf2::convert(pose2.pose.orientation, q2);

    const double da = angular_distance_weight * std::abs(q1.angleShortestPath(q2));
    const double dp = std::sqrt(dx * dx + dy * dy + dz * dz);
    return std::sqrt(dp * dp + da * da);
}

void TruncatePathLocal::resamplePath(nav_msgs::msg::Path& path, double spacing_m) {
    if (spacing_m <= 0.0 || path.poses.size() < 3) {
        return;
    }

    std::vector<geometry_msgs::msg::PoseStamped> out;
    out.reserve(path.poses.size());
    out.push_back(path.poses.front());

    double acc = 0.0;
    for (size_t i = 1; i < path.poses.size(); ++i) {
        acc += nav3d_util::geometry_utils::euclidean_distance(path.poses[i - 1], path.poses[i], true);
        if (acc >= spacing_m || i + 1 == path.poses.size()) {
            out.push_back(path.poses[i]);
            acc = 0.0;
        }
    }

    path.poses = std::move(out);
}

void TruncatePathLocal::capPath(nav_msgs::msg::Path& path, int max_poses) {
    if (max_poses <= 0 || static_cast<int>(path.poses.size()) <= max_poses) {
        return;
    }

    const size_t n = path.poses.size();
    const size_t last = n - 1;
    const size_t keep = static_cast<size_t>(max_poses);

    std::vector<geometry_msgs::msg::PoseStamped> out;
    out.reserve(keep);

    if (keep == 1) {
        out.push_back(path.poses[last]);
        path.poses = std::move(out);
        return;
    }

    out.push_back(path.poses.front());

    const double step = static_cast<double>(last) / static_cast<double>(keep - 1);
    size_t prev = 0;

    for (size_t i = 1; i + 1 < keep; ++i) {
        size_t idx = static_cast<size_t>(std::round(i * step));
        if (idx <= prev) {
            idx = prev + 1;
        }
        if (idx >= last) {
            idx = last - 1;
        }
        out.push_back(path.poses[idx]);
        prev = idx;
    }

    out.push_back(path.poses[last]);
    path.poses = std::move(out);
}

} // namespace nav3d_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) { factory.registerNodeType<nav3d_behavior_tree::TruncatePathLocal>("TruncatePathLocal"); }