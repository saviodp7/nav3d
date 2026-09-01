#pragma once

#include <string>
#include <vector>

namespace nav3d_util {

using Tokens = std::vector<std::string>;

[[nodiscard]] std::string strip_leading_slash(const std::string& in);

[[nodiscard]] Tokens split(const std::string& tokenstring, char delimiter);

} // namespace nav3d_util
