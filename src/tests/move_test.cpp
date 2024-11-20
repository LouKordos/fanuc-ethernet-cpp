#include "../include/fanuc-ethernet.hpp"
#include <print>

int main() {
    fanuc_ethernet::FANUCRobot robot{"10.36.12.3"};
    std::println("Robot object initialized");
    auto init_result = robot.initialize_connection();

    if(!init_result.has_value()) {
        std::println("Failed to connect to FANUC robot, exiting: {0}", init_result.error());
        return 1;
    }

    for(int i = 0; i < 1000; i++) {
        auto result = robot.read_R_register(1);
        if(result.has_value()) {
            std::println("Value: {0}", result.value());        
        }
        else {
            std::println("No value returned for register, exiting: {0}", result.error());
            return 1;
        }
    }
    
    auto success = robot.write_R_register(1, 69);
    if(success) {
        std::println("Position write succeeded.");
    }
    else {
        std::println("Position write failed, exiting: {0}", success.error());
        return 1;
    }

    for(int i = 0; i < 1000; i++) {
        std::expected<fanuc_ethernet::robot_pose, std::string> result = robot.read_current_position();
        if(result.has_value()) {
            fanuc_ethernet::robot_pose pose = result.value();
            std::println("Robot current position read succeeded: UTOOL={0}, UFRAME={1}, X={2}, Y={3}, Z={4}, Yaw={5}, Pitch={6}, Roll={7}", pose.utool, pose.uframe, pose.x, pose.y, pose.z, pose.yaw, pose.pitch, pose.roll);
        }
    }

    for(int i = 0; i < 1000; i++) {
        const uint reg_num = 4;
        std::expected <fanuc_ethernet::robot_pose, std::string> result = robot.read_PR_register(reg_num);
        if(result.has_value()) {
            fanuc_ethernet::robot_pose pose = result.value();
            std::println("Robot PR[{8}] read succeeded: UTOOL={0}, UFRAME={1}, X={2}, Y={3}, Z={4}, Yaw={5}, Pitch={6}, Roll={7}", pose.utool, pose.uframe, pose.x, pose.y, pose.z, pose.yaw, pose.pitch, pose.roll, reg_num);
        }
    }
}