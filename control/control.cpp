#include <chrono>
#include <iostream>
#include <thread>

#include "drone_lib.hpp"

int main() {
    try {
        YamlValue settings = drone::load_mavlink_settings();
        drone::connect(settings["real"]["proxy_udp"]["address"].as_string());
        const double target_alt = 10;

        drone::set_mode("GUIDED");
        std::this_thread::sleep_for(std::chrono::seconds(2));

        drone::arm_disarm(true);
        std::cout << "시동 완료." << std::endl;

        drone::takeoff(target_alt);

        drone::land();
        std::cout << "착륙 중..." << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
}
