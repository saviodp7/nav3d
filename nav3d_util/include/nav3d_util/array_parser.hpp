#pragma once

#include <string>
#include <vector>

namespace nav3d_util {
[[nodiscard]] std::vector<std::vector<float>> parseVVF(const std::string& input, std::string& error);
} // namespace nav3d_util
