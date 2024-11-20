#ifndef FANUC_ETHERNET_CPP
#define FANUC_ETHERNET_CPP

#include <EIPScanner/MessageRouter.h>
#include <EIPScanner/utils/Logger.h>
#include <EIPScanner/utils/Buffer.h>
#include <iostream>
#include <expected>
#include <string>
#include <unistd.h>

#include <tracy/Tracy.hpp>

using eipScanner::SessionInfo;
using eipScanner::MessageRouter;
using namespace eipScanner::cip;
using namespace eipScanner::utils;

namespace fanuc_ethernet {
    struct robot_pose {
        short utool;
        short uframe;
        float x;
        float y;
        float z;
        float yaw;
        float pitch;
        float roll;
        uint8_t turn1;
        uint8_t turn2;
        uint8_t turn3;
        uint8_t bitflip;
        float E0;
        float E1;
        float E2;
    };

    // THIS LIRBARY IS NOT THREADSAFE AND DOES NOT PROVIDE ANY GUARANTEES WHEN ACCESSED BY MULTIPLE THREADS
    class FANUCRobot {
        private:
            std::string ip;
            std::shared_ptr<MessageRouter> message_router;
            std::shared_ptr<SessionInfo> session_info;
            uint timeout_milliseconds;
            bool connected {false};
            bool fine_high_speed_skip_enabled {false};
            const uint ENABLE_REGISTER = 1; // Enable Register, 1 means robot moves, 0 means robot does not move, and interrupts using high speed skip
            const uint HIGH_SPEED_SKIP_FLAG_REGISTER = 21; // Switch case register to determine mode, 0 is FINE + High speed Skip, 1 is CNT100 + "normal skip"
            const uint VELOCITY_LIMIT_REGISTER = 5; // Velocity register, used in move commands to control the velocity
            const uint POSE_SETPOINT_POSITION_REGISTER = 1; // Position setpoint used by TP program
            const uint SKIP_POSE_POSITION_REGISTER = 4; // Set by high speed skip to position when it skips
        public:
            FANUCRobot(const std::string &ip, uint timeout_milliseconds = 30) : ip(ip), timeout_milliseconds(timeout_milliseconds) {
                
            }

            std::expected<void, std::string> initialize_connection() {
                ZoneScoped;
                try {
                    // Open EIP session with the adapter, 0xAF12 is default CIP/EIP port
                    session_info = std::make_shared<SessionInfo>(ip, 0xAF12, std::chrono::milliseconds(timeout_milliseconds));
                    message_router = std::make_shared<MessageRouter>();
                    connected = true;
                    return {};
                }
                catch(std::exception &e) {
                    return std::unexpected{e.what()};
                }
            }

            std::expected<void, std::string> write_R_register(const uint register_index, const int32_t value) {
                ZoneScoped;
                if(!connected) {
                    return std::unexpected{"Not connected yet."};
                }
                try {
                    const CipUsint service_id = 0x10;
                    const EPath epath(0x6B, 0x1, register_index);
                    Buffer data;
                    data << value;

                    auto response = message_router->sendRequest(session_info, service_id, epath, data.data());

                    if (response.getGeneralStatusCode() == GeneralStatusCodes::SUCCESS) {
                        return {};
                    }
                    else {
                        return std::unexpected{std::to_string(static_cast<int>(response.getGeneralStatusCode()))};
                    }
                } catch (std::exception &e) {
                    return std::unexpected{e.what()};
                }
            }

            std::expected<int, std::string> read_R_register(const uint register_index) {
                ZoneScoped;
                if(!connected) {
                    return std::unexpected{"Not connected yet."};
                }
                try {
                    const CipUsint service_id = 0xe;
                    const EPath epath(0x6B, 0x1, register_index);

                    auto response = message_router->sendRequest(session_info, service_id, epath);

                    if (response.getGeneralStatusCode() == GeneralStatusCodes::SUCCESS) {
                        Buffer buffer(response.getData());
                        CipInt result;
                        buffer >> result;
                        return result;
                    }
                    else {
                        return std::unexpected{std::to_string(static_cast<int>(response.getGeneralStatusCode()))};
                    }
                } catch (std::exception &e) {
                    return std::unexpected{e.what()};
                }
            }

            std::expected<float, std::string> parse_float_from_buffer(const std::vector<uint8_t> &buf, const int offset) {
                ZoneScoped;
                if(buf.size() < offset + sizeof(float)) {
                    return std::unexpected{"Buffer too small to parse float"};
                }

                float temp;
                std::memcpy(&temp, &buf[offset], sizeof(temp));
                return temp;

                // Risky due to alignment and endianness:
                // return *reinterpret_cast<const float*>(&buf[offset]);
            }

            std::expected<uint8_t, std::string> parse_byte_from_buffer(const std::vector<uint8_t> &buf, const int offset) {
                ZoneScoped;
                if(buf.size() < offset + sizeof(uint8_t)) {
                    return std::unexpected{"Buffer too small to parse byte"};
                }

                uint8_t temp;
                std::memcpy(&temp, &buf[offset], sizeof(temp));
                return temp;
            }

            std::expected<robot_pose, std::string> parse_pose_buffer(const std::vector<uint8_t> &buf) {
                ZoneScoped;
                robot_pose pose {};

                pose.utool = buf[0] + buf[1] * 8;
                pose.uframe = buf[2] + buf[3] * 8;
                
                int offset {4};

                auto result = parse_float_from_buffer(buf, offset);
                if(!result.has_value()) {
                    return std::unexpected{result.error()}; 
                }
                pose.x = result.value();
                offset += sizeof(pose.x);

                result = parse_float_from_buffer(buf, offset);
                if(!result.has_value()) {
                    return std::unexpected{result.error()}; 
                }
                pose.y = result.value();
                offset += sizeof(pose.y);
   
                result = parse_float_from_buffer(buf, offset);
                if(!result.has_value()) {
                    return std::unexpected{result.error()}; 
                }
                pose.z = result.value();
                offset += sizeof(pose.z);

                result = parse_float_from_buffer(buf, offset);
                if(!result.has_value()) {
                    return std::unexpected{result.error()}; 
                }
                pose.yaw = result.value();
                offset += sizeof(pose.yaw);

                result = parse_float_from_buffer(buf, offset);
                if(!result.has_value()) {
                    return std::unexpected{result.error()}; 
                }
                pose.pitch = result.value();
                offset += sizeof(pose.pitch);

                result = parse_float_from_buffer(buf, offset);
                if(!result.has_value()) {
                    return std::unexpected{result.error()}; 
                }
                pose.roll = result.value();
                offset += sizeof(pose.roll);

                auto byte_result = parse_byte_from_buffer(buf, offset);
                if(!result.has_value()) {
                    return std::unexpected{result.error()}; 
                }
                pose.turn1 = byte_result.value();
                offset += sizeof(pose.turn1);

                byte_result = parse_byte_from_buffer(buf, offset);
                if(!result.has_value()) {
                    return std::unexpected{result.error()}; 
                }
                pose.turn2 = byte_result.value();
                offset += sizeof(pose.turn2);

                byte_result = parse_byte_from_buffer(buf, offset);
                if(!result.has_value()) {
                    return std::unexpected{result.error()}; 
                }
                pose.turn3 = byte_result.value();
                offset += sizeof(pose.turn3);

                byte_result = parse_byte_from_buffer(buf, offset);
                if(!result.has_value()) {
                    return std::unexpected{result.error()}; 
                }
                pose.bitflip = byte_result.value();
                offset += sizeof(pose.bitflip);

                result = parse_float_from_buffer(buf, offset);
                if(!result.has_value()) {
                    return std::unexpected{result.error()}; 
                }
                pose.E0 = result.value();
                offset += sizeof(pose.E0);

                result = parse_float_from_buffer(buf, offset);
                if(!result.has_value()) {
                    return std::unexpected{result.error()}; 
                }
                pose.E1 = result.value();
                offset += sizeof(pose.E1);

                result = parse_float_from_buffer(buf, offset);
                if(!result.has_value()) {
                    return std::unexpected{result.error()}; 
                }
                pose.E2 = result.value();
                offset += sizeof(pose.E2);

                return pose;
            }

            std::expected<robot_pose, std::string> read_current_pose() {
                ZoneScoped;
                if(!connected) {
                    return std::unexpected{"Not connected yet."};
                }
                try {
                    const CipUsint service_id = ServiceCodes::GET_ATTRIBUTE_SINGLE;
                    const EPath epath(0x7D, 0x01, 0x01);

                    auto response = message_router->sendRequest(session_info, service_id, epath);

                    if (response.getGeneralStatusCode() == GeneralStatusCodes::SUCCESS) {
                        Buffer buffer(response.getData());
                        return parse_pose_buffer(buffer.data());
                    }
                    else {
                        return std::unexpected{"Error: " + std::to_string(static_cast<int>(response.getGeneralStatusCode()))};
                    }
                } catch (std::exception &e) {
                    return std::unexpected{"Exception:" + std::string{e.what()}};
                }
            }

            std::expected<robot_pose, std::string> read_PR_register(const uint register_index) {
                ZoneScoped;
                if(!connected) {
                    return std::unexpected{"Not connected yet."};
                }
                try {
                    const CipUsint service_id = ServiceCodes::GET_ATTRIBUTE_SINGLE;
                    const EPath epath(0x7B, 0x01, register_index);

                    auto response = message_router->sendRequest(session_info, service_id, epath);

                    if (response.getGeneralStatusCode() == GeneralStatusCodes::SUCCESS) {
                        Buffer buffer(response.getData());
                        return parse_pose_buffer(buffer.data());
                    }
                    else {
                        return std::unexpected{"Error: " + std::to_string(static_cast<int>(response.getGeneralStatusCode()))};
                    }
                } catch (std::exception &e) {
                    return std::unexpected{"Exception:" + std::string{e.what()}};
                }
            }

            std::expected<void, std::string> write_PR_register(const uint register_index, const fanuc_ethernet::robot_pose &desired_pose) {
                ZoneScoped;
                if(!connected) {
                    return std::unexpected{"Not connected yet."};
                }
                try {
                    const CipUsint service_id = 0x10;
                    const EPath epath(0x7B, 0x01, register_index);
                    Buffer data;
                    // Apparently UFRAME and UTOOL have to always be 0.
                    data << (short)0 << (short)0 << desired_pose.x << desired_pose.y << desired_pose.z << desired_pose.yaw << desired_pose.pitch << desired_pose.roll
                        << desired_pose.turn1 << desired_pose.turn2 << desired_pose.turn3 << desired_pose.bitflip << desired_pose.E0 << desired_pose.E1 << desired_pose.E2;

                    // std::cout << data.data().size()<<std::endl;

                    auto response = message_router->sendRequest(session_info, service_id, epath, data.data());

                    if (response.getGeneralStatusCode() == GeneralStatusCodes::SUCCESS) {
                        return {};
                    }
                    else {
                        return std::unexpected{std::to_string(static_cast<int>(response.getGeneralStatusCode()))};
                    }
                } catch (std::exception &e) {
                    return std::unexpected{e.what()};
                }
            }

            std::expected<void, std::string> enable_robot() {
                ZoneScoped;
                return write_R_register(ENABLE_REGISTER, 1);
            }

            std::expected<void, std::string> disable_robot() {
                ZoneScoped;
                return write_R_register(ENABLE_REGISTER, 0);
            }

            // See register-definition.txt
            // Ideally we would add another register that allows to check if the robot is moving, so that these concerns are separated.
            // Currently, the same register is used for both enabled and moving, which is suboptimal.
            // To fix this, a second register (moving_register) would have to be set when the movement is finished and the main loop would have to use the enable_register.
            std::expected<bool, std::string> is_enabled() {
                ZoneScoped;
                auto res = read_R_register(ENABLE_REGISTER);
                if(res.has_value()) {
                    return res.value() == 1;
                }
                else return std::unexpected {res.error()};
            }

            // Sets movement mode to FINE and enables high speed skip. This should be used for final adjustments as it's limited to 100mm/s.
            // If controlling FINE/CNT and high speed skip separately is desired, the register could be used to store values other than 0 and 1.
            // However, the TP program needs to be adjusted accordingly in that case, of course.
            std::expected<void, std::string> set_mode_fine_high_speed_skip() {
                ZoneScoped;
                auto res = write_R_register(HIGH_SPEED_SKIP_FLAG_REGISTER, 0);
                if(res.has_value()) {
                    fine_high_speed_skip_enabled = true;
                    return {};
                }
                else {
                    return res;
                }
            }

            // Sets movement mode to CNT and switches to "normal speed" skip.
            // If controlling FINE/CNT and high speed skip separately is desired, the register could be used to store values other than 0 and 1.
            // However, the TP program needs to be adjusted accordingly in that case, of course.
            std::expected<void, std::string> set_mode_cnt_normal_skip() {
                ZoneScoped;
                auto res = write_R_register(HIGH_SPEED_SKIP_FLAG_REGISTER, 1);
                if(res.has_value()) {
                    fine_high_speed_skip_enabled = false;
                    return {};
                }
                else {
                    return res;
                }
            }

            bool is_fine_high_speed_skip_enabled() {
                return fine_high_speed_skip_enabled;
            }

            std::expected<void, std::string> set_velocity_limit(const uint vel_limit) {
                ZoneScoped;
                return write_R_register(VELOCITY_LIMIT_REGISTER, vel_limit);
            }

            // THIS IS VERY DANGEROUS AS THE ROBOT WILL NOT STOP UNTIL THE SERVOS BREAK, AS NO FORCE LIMITS ARE CHECKED WHILE MOVING!!!
            // IMPLEMENT YOUR OWN FORCE CHECKS ON A SEPARATE THREAD AND USE THE ASYNC VERSION OF THIS FUNCTION!!!
            std::expected<void, std::string> move_to_pos_sync(const fanuc_ethernet::robot_pose &desired_pose) {
                ZoneScoped;
                if(const auto res = write_PR_register(POSE_SETPOINT_POSITION_REGISTER, desired_pose); res.has_value()) {
                    std::cout << "PR write succeeded, enabling robot...\n";
                    this->enable_robot();
                }
                else return res;

                // usleep(1000);

                // Spin until movement is finished
                while(true) {
                    ZoneScoped;
                    const auto res = this->is_enabled();
                    if(res.has_value()) {
                        if(!res.value()) {
                            std::cout << "Movement finished, returning.\n";
                            return {};
                        }
                        std::cout << "Waiting for movement to finish...\n";
                    }
                    else {
                        return std::unexpected{res.error()};
                    }
                }
            }

            // THIS IS VERY DANGEROUS AS THE ROBOT WILL NOT STOP UNTIL THE SERVOS BREAK OR THE POSITION IS REACHED. NO FORCE LIMITS ARE CHECKED WHILE MOVING!!!
            // IMPLEMENT YOUR OWN FORCE CHECKS ON A SEPARATE THREAD AND USE THE ASYNC VERSION OF THIS FUNCTION!!!
            // THIS FUNCTION WILL RETURN IMMEDIATELY AND ONLY TELL THE ROBOT TO MOVE TO THE DESIRED POSE WITHOUT BLOCKING IF IT IS DONE!
            // Also, you have to specify stop_current_movement = true so that it actually stops the robot, updates the setpoint, and moves to the desired position.
            // Otherwise, the robot MIGHT move to the first (potentially outdated) setpoint and only after that start moving to the newly set one.
            std::expected<void, std::string> move_to_pos_async(const fanuc_ethernet::robot_pose &desired_pose, bool stop_current_movement = false) {
                ZoneScoped;
                if(stop_current_movement) {
                    std::cout << "Stop current movement specified, stopping before updating register...\n";
                    auto res = this->disable_robot();
                    if(!res.has_value()) {
                        std::cout << "Disabling robot failed!\n";
                        return std::unexpected{res.error()};
                    }
                }
                if(const auto res = write_PR_register(POSE_SETPOINT_POSITION_REGISTER, desired_pose); res.has_value()) {
                    std::cout << "PR write succeeded, enabling robot...\n";
                    return this->enable_robot();
                }
                else return res;
            }
    };
}

#endif