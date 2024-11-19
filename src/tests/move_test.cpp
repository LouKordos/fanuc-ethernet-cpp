#include "../include/fanuc-ethernet.hpp"
#include <print>

int main() {
    fanuc_ethernet::FANUCRobot robot{"10.36.12.3"};
    std::println("Robot object initialized");

    if(!robot.initialize_connection()) {
        std::println("Failed to connect to FANUC robot, exiting.");
        return 1;
    }

    for(int i = 0; i < 1000; i++) {
    auto result = robot.read_R_register(1);
    if(result.has_value()) {
        std::println("Value: {0}", result.value());        
    }
    else {
        std::println("No value returned for register, exiting.");
        return 1;
    }
    }
    return 0;
}