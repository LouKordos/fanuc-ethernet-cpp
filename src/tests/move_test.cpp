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
    
    auto success = robot.write_R_register(1, 69);
    if(success) {
        std::println("Position write succeeded.");
    }
    else {
        std::println("Position write failed, exiting.");
        return 1;
    }

    std::expected<fanuc_ethernet::robot_pose, std::string> result = robot.read_current_position();
    if(result.has_value()) {
        fanuc_ethernet::robot_pose pose = result.value();
        std::println("Robot current position read suceeded: UTOOL={0}, UFRAME={1}, X={2}, Y={3}, Z={4}, Yaw={5}, Pitch={6}, Roll={7}", pose.utool, pose.uframe, pose.x, pose.y, pose.z, pose.yaw, pose.pitch, pose.roll);
    }
}