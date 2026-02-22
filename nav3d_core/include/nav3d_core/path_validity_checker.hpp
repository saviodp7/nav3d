#pragma once

#include <cstdint>
#include <vector>

#include "nav_msgs/msg/path.hpp"

namespace nav3d_core
{

class PathValidityChecker
{
public:
  virtual ~PathValidityChecker() = default;

  virtual bool isPathValid(
    const nav_msgs::msg::Path & path,
    std::vector<int32_t> & invalid_pose_indices) = 0;
};

}  // namespace nav3d_core