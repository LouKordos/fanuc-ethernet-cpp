#ifndef FANUC_ETHERNET_CPP
#define FANUC_ETHERNET_CPP

#include <EIPScanner/MessageRouter.h>
#include <EIPScanner/utils/Logger.h>
#include <EIPScanner/utils/Buffer.h>
#include <iostream>
#include <expected>
#include <string>
#include <tracy/Tracy.hpp>

using eipScanner::SessionInfo;
using eipScanner::MessageRouter;
using namespace eipScanner::cip;
using namespace eipScanner::utils;

// TODO: Include Tracy and measure send and connect times in detail, add message text to profiler metadata

namespace fanuc_ethernet {
    struct robot_pose {
        int utool;
        int uframe;
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

    class FANUCRobot {
        private:
            std::string ip;
            std::shared_ptr<MessageRouter> message_router;
            std::shared_ptr<SessionInfo> session_info;
            bool connected {false};
        public:
            FANUCRobot(const std::string &ip) : ip(ip) {
                
            }

            std::expected<void, std::string> initialize_connection() {
                ZoneScoped;
                try {
                    // Open EIP session with the adapter, 0xAF12 is default CIP/EIP port
                    session_info = std::make_shared<SessionInfo>(ip, 0xAF12);
                    message_router = std::make_shared<MessageRouter>();
                    connected = true;
                    return {};
                }
                catch(std::exception &e) {
                    return std::unexpected{e.what()};
                }
            }

            std::expected<void, std::string> write_R_register(uint register_index, int32_t value) {
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

            std::expected<int, std::string> read_R_register(uint register_index) {
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

            std::expected<float, std::string> parse_float_from_buffer(const std::vector<uint8_t> &buf, int offset) {
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

            std::expected<robot_pose, std::string> parse_pose_buffer(const std::vector<uint8_t> &buf) {
                ZoneScoped;
                robot_pose pose;

                pose.utool = buf[0] + buf[1] * 8;
                pose.uframe = buf[2] + buf[3] * 8;
                
                int offset = 4;

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

                return pose;
            }

            std::expected<robot_pose, std::string> read_current_position() {
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

            // TODO: enable_robot, disable_robot, is_enabled, is_moving?, write_position_register(register_index), set_mode_cnt_normal_skip, set_mode_fine_high_speed_skip, 
            // set_velocity_limit, move_to_pos_sync(x,y,z,yaw,pitch,roll), move_to_pos_async(x,y,z,yaw,pitch,roll)
    };
}

#endif