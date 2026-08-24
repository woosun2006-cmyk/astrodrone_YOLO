#include "flight_mission_app.hpp"

#include <iostream>
#include <string>

namespace {

bool parse_options(int argc, char** argv, app::FlightMissionAppOptions& options) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg(argv[i]);
        if (arg == "--self-launch") {
            options.self_launch = true;
        } else if (arg == "--auto-intercept") {
            options.auto_intercept = true;
        } else {
            std::cerr << "알 수 없는 옵션: " << arg
                      << " (지원: --auto-intercept, --self-launch)" << std::endl;
            return false;
        }
    }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    app::FlightMissionAppOptions options;
    if (!parse_options(argc, argv, options)) return 2;

    try {
        return app::FlightMissionApp(options).run();
    } catch (const std::exception& e) {
        std::cerr << "[실패] " << e.what() << std::endl;
        return 1;
    }
}
