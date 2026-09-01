#include "nav3d_behavior_tree/plugins/action/publish_path_action.hpp"

#include <memory>
#include <string>

namespace nav3d_behavior_tree {

PublishPath::PublishPath(const std::string& name, const BT::NodeConfiguration& conf) : BT::ActionNodeBase(name, conf) {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    getInput("topic_name", topic_name_);
    getInput("queue_size", queue_size_);
    ensurePublisher();
}

void PublishPath::ensurePublisher() {
    if (!node_) {
        return;
    }
    if (!pub_ || pub_->get_topic_name() != (std::string("/") + topic_name_)) {
        rclcpp::QoS qos(std::max(1, queue_size_));
        pub_ = node_->create_publisher<nav_msgs::msg::Path>(topic_name_, qos);
    }
}

BT::NodeStatus PublishPath::tick() {
    nav_msgs::msg::Path path;
    if (!getInput("path", path)) {
        return BT::NodeStatus::FAILURE;
    }

    std::string topic = topic_name_;
    int qs = queue_size_;

    getInput("topic_name", topic);
    getInput("queue_size", qs);

    if (topic != topic_name_ || qs != queue_size_) {
        topic_name_ = std::move(topic);
        queue_size_ = qs;
        pub_.reset();
        ensurePublisher();
    }

    if (!pub_) {
        return BT::NodeStatus::FAILURE;
    }

    pub_->publish(path);
    return BT::NodeStatus::SUCCESS;
}

} // namespace nav3d_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) { factory.registerNodeType<nav3d_behavior_tree::PublishPath>("PublishPath"); }