#include "nav3d_util/array_parser.hpp"

#include <cstdio>
#include <sstream>
#include <string>
#include <vector>

namespace nav3d_util {

std::vector<std::vector<float>> parseVVF(const std::string& input, std::string& error_return) {
    std::vector<std::vector<float>> result;

    std::stringstream input_ss(input);
    int depth = 0;
    std::vector<float> current_vector;

    while (static_cast<bool>(input_ss) && !input_ss.eof()) {
        switch (input_ss.peek()) {
        case EOF:
            break;

        case '[':
            ++depth;
            if (depth > 2) {
                error_return = "Array depth greater than 2";
                return result;
            }
            input_ss.get();
            current_vector.clear();
            break;

        case ']':
            --depth;
            if (depth < 0) {
                error_return = "More close ] than open [";
                return result;
            }
            input_ss.get();
            if (depth == 1) {
                result.push_back(current_vector);
            }
            break;

        case ',':
        case ' ':
        case '\t':
            input_ss.get();
            break;

        default:
            if (depth != 2) {
                std::stringstream err_ss;
                err_ss << "Numbers at depth other than 2. Char was '" << char(input_ss.peek()) << "'.";
                error_return = err_ss.str();
                return result;
            }

            float value;
            input_ss >> value;
            if (!static_cast<bool>(input_ss)) {
                error_return = "Failed to parse float value.";
                return result;
            }
            current_vector.push_back(value);
            break;
        }
    }

    if (depth != 0) {
        error_return = "Unterminated vector string.";
        return result;
    }

    error_return.clear();
    return result;
}

} // namespace nav3d_util
