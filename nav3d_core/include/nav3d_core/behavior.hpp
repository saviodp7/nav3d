#pragma once

#include <memory>
#include <string>

namespace rclcpp_lifecycle {
class LifecycleNode;
}

namespace tf2_ros {
class Buffer;
}

namespace nav3d_core {

enum class CostmapInfoType { NONE = 0, LOCAL = 1, GLOBAL = 2, BOTH = 3 };

class Behavior {
  public:
    using Ptr = std::shared_ptr<Behavior>;

    virtual ~Behavior() noexcept = default;

    virtual void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, const std::string& name,
                           std::shared_ptr<tf2_ros::Buffer> tf) = 0;

    virtual void cleanup() = 0;
    virtual void activate() = 0;
    virtual void deactivate() = 0;

    virtual CostmapInfoType getResourceInfo() = 0;
};

} // namespace nav3d_core
