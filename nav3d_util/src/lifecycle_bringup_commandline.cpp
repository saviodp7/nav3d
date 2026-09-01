#include "nav3d_util/lifecycle_utils.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cstdlib>
#include <string>
#include <vector>

using namespace std::chrono_literals;

[[noreturn]] static void usage() {
    std::cerr << "Invalid command line.\n\n";
    std::cerr << "This command will take a set of unconfigured lifecycle nodes through the\n";
    std::cerr << "CONFIGURED to the ACTIVATED state\n";
    std::cerr << "The nodes are brought up in the order listed on the command line\n\n";
    std::cerr << "Usage:\n";
    std::cerr << " > lifecycle_startup <node name> ...\n";
    std::exit(1);
}

int main(int argc, char* argv[]) {
    if (argc <= 1) {
        usage();
    }

    rclcpp::init(argc, argv);

    nav3d_util::startup_lifecycle_nodes(std::vector<std::string>(argv + 1, argv + argc), 10s);

    rclcpp::shutdown();
    return 0;
}
