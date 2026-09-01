#include "nav3d_util/node_thread.hpp"

#include <memory>
#include <thread>

namespace nav3d_util {

NodeThread::NodeThread(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base) : node_(std::move(node_base)) {
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    thread_ = std::make_unique<std::thread>([this]() {
        executor_->add_node(node_);
        executor_->spin();
        executor_->remove_node(node_);
    });
}

NodeThread::NodeThread(rclcpp::executors::SingleThreadedExecutor::SharedPtr executor) : executor_(std::move(executor)) {
    thread_ = std::make_unique<std::thread>([this]() { executor_->spin(); });
}

NodeThread::~NodeThread() {
    if (executor_) {
        executor_->cancel();
    }
    if (thread_ && thread_->joinable()) {
        thread_->join();
    }
}

} // namespace nav3d_util
