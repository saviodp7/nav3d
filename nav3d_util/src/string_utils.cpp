#include "nav3d_util/string_utils.hpp"

#include <string>

namespace nav3d_util {

std::string strip_leading_slash(const std::string& in) {
    size_t i = 0;
    while (i < in.size() && in[i] == '/') {
        ++i;
    }
    return in.substr(i);
}

Tokens split(const std::string& tokenstring, char delimiter) {
    Tokens tokens;

    size_t current_pos = 0;
    size_t pos = 0;
    while ((pos = tokenstring.find(delimiter, current_pos)) != std::string::npos) {
        tokens.push_back(tokenstring.substr(current_pos, pos - current_pos));
        current_pos = pos + 1;
    }
    tokens.push_back(tokenstring.substr(current_pos));
    return tokens;
}

} // namespace nav3d_util
