#pragma once

#include "nav_msgs/msg/path.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <memory>
#include <string>

namespace nav3d_core {

class Smoother {
  public:
    using Ptr = std::shared_ptr<Smoother>;

    virtual ~Smoother() = default;

    virtual void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, const std::string& name) = 0;

    virtual void cleanup() = 0;
    virtual void activate() = 0;
    virtual void deactivate() = 0;

    virtual bool smooth(nav_msgs::msg::Path& path, const rclcpp::Duration& max_time) = 0;
};

} // namespace nav3d_core
