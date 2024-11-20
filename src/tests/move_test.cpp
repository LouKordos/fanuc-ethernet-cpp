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
            std::println("Robot current pose read succeeded: UTOOL={0}, UFRAME={1}, X={2}, Y={3}, Z={4}, Yaw={5}, Pitch={6}, Roll={7}, turn1={8}, turn2={9}, turn3={10}, bitflip={11}, E0={12}, E1={13}, E2={14}", pose.utool, pose.uframe, pose.x, pose.y, pose.z, pose.yaw, pose.pitch, pose.roll, pose.turn1, pose.turn2, pose.turn3, pose.bitflip, pose.E0, pose.E1, pose.E2);
        }
        else {
            std::println("Robot current pose read failed, exiting: {0}", result.error());
            return 1;
        }
    }

    fanuc_ethernet::robot_pose desired_pose {};

    for(int i = 0; i < 1000; i++) {
        const uint reg_num = 4;
        std::expected <fanuc_ethernet::robot_pose, std::string> result = robot.read_PR_register(reg_num);
        if(result.has_value()) {
            fanuc_ethernet::robot_pose pose = result.value();
            desired_pose = pose;
            std::println("Robot PR read succeeded: UTOOL={0}, UFRAME={1}, X={2}, Y={3}, Z={4}, Yaw={5}, Pitch={6}, Roll={7}, turn1={8}, turn2={9}, turn3={10}, bitflip={11}, E0={12}, E1={13}, E2={14}", pose.utool, pose.uframe, pose.x, pose.y, pose.z, pose.yaw, pose.pitch, pose.roll, pose.turn1, pose.turn2, pose.turn3, pose.bitflip, pose.E0, pose.E1, pose.E2);
        }
        else {
            std::println("Robot PR[{0}] read failed, exiting: {1}", reg_num, result.error());
            return 1;
        }
    }
    
    for(int i = 0; i < 1000; i++) {
        const uint reg_num = 6;

        desired_pose.x = 312.33;
        desired_pose.y = 313.44;
        desired_pose.z = 314.55;
        desired_pose.yaw = 1.69;
        desired_pose.pitch = 2.69;
        desired_pose.roll = 3.69;

        auto result = robot.write_PR_register(reg_num, desired_pose);
        if(result.has_value()) {
            std::println("Robot PR[{8}] write succeeded, check values on FANUC TP: UTOOL={0}, UFRAME={1}, X={2}, Y={3}, Z={4}, Yaw={5}, Pitch={6}, Roll={7}", desired_pose.utool, desired_pose.uframe, desired_pose.x, desired_pose.y, desired_pose.z, desired_pose.yaw, desired_pose.pitch, desired_pose.roll, reg_num);
        }
        else {
            std::println("Robot PR[{0}] write failed, exiting: {1}", reg_num, result.error());
            return 1;
        }
    }
}