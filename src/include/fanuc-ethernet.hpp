#ifndef FANUC_ETHERNET_CPP
#define FANUC_ETHERNET_CPP

#include <MessageRouter.h>
#include <utils/Logger.h>
#include <utils/Buffer.h>
#include <iostream>
#include <expected>
#include <string>
#include <unistd.h>

#include <tracy/Tracy.hpp>

namespace fanuc_ethernet {
    /**
 	* @brief Represents the pose of the robot in terms of position and orientation.
 	*
 	* This structure contains all the necessary parameters to define
 	* the robot's pose, including position coordinates, orientation angles,
 	* tool and frame references, and additional parameters.
 	*/
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

    /**
 	* @brief Provides an interface to control a FANUC robot via Ethernet/IP.
 	*
 	* This class emulates a PLC to directly interact with the robot controller's registers,
 	* allowing control over position, speed, and various modes. It is important to note that
 	* this library is not thread-safe and should be accessed from a single thread.
 	*/
    class FANUCRobot {
        private:
            std::string ip;
            std::shared_ptr<eipScanner::MessageRouter> message_router;
            std::shared_ptr<eipScanner::SessionInfo> session_info;
            uint16_t timeout_milliseconds;
            bool connected {false};
            bool fine_high_speed_skip_enabled {false};
            const uint8_t ENABLE_REGISTER = 1; // Enable Register, 1 means robot moves, 0 means robot does not move, and interrupts using high speed skip
            const uint8_t HIGH_SPEED_SKIP_FLAG_REGISTER = 21; // Switch case register to determine mode, 0 is FINE + High speed Skip, 1 is CNT100 + "normal skip"
            const uint8_t VELOCITY_LIMIT_REGISTER = 5; // Velocity register, used in move commands to control the velocity
            const uint8_t POSE_SETPOINT_POSITION_REGISTER = 1; // Position setpoint used by TP program
            const uint8_t SKIP_POSE_POSITION_REGISTER = 4; // Set by high speed skip to position when it skips
        public:
            /**
            * @brief Constructs a FANUCRobot object with the given IP address and timeout.
            * @param ip The IP address of the robot controller.
            * @param timeout_milliseconds Communication timeout in milliseconds (default is 50 ms).
            */
            FANUCRobot(const std::string &ip, eipScanner::utils::LogLevel log_level, uint16_t timeout_milliseconds = 50) : ip(ip), timeout_milliseconds(timeout_milliseconds) {
                eipScanner::utils::Logger::setLogLevel(log_level);
            }

            /**
            * @brief Initializes the connection to the robot controller.
            *
            * This function establishes an Ethernet/IP session with the robot controller.
            * @return An expected containing void on success, or an error message on failure.
            */
            std::expected<void, std::string> initialize_connection() {
                ZoneScoped;
                try {
                    // Open EIP session with the adapter, 0xAF12 is default CIP/EIP port
                    session_info = std::make_shared<eipScanner::SessionInfo>(ip, 0xAF12, std::chrono::milliseconds(timeout_milliseconds));
                    message_router = std::make_shared<eipScanner::MessageRouter>();
                    connected = true;
                    return {};
                }
                catch(std::exception &e) {
                    return std::unexpected{e.what()};
                }
            }

            /**
            * @brief Writes an integer value to an R register on the robot controller.
            * @param register_index The index of the R register to write to.
            * @param value The integer value to write to the register.
            * @return An expected containing void on success, or an error message on failure. Resource temporarily unavailable error might mean that timeout was exceeded. You can specify this timeout in the FANUCRobot constructor.
            */
            std::expected<void, std::string> write_R_register(const uint8_t register_index, const int32_t value) {
                ZoneScoped;
                if(!connected) {
                    return std::unexpected{"Not connected yet."};
                }
                try {
                    const eipScanner::cip::CipUsint service_id = 0x10;
                    const eipScanner::cip::EPath epath(0x6B, 0x1, register_index);
                    eipScanner::utils::Buffer data;
                    data << value;

                    auto response = message_router->sendRequest(session_info, service_id, epath, data.data());

                    if (response.getGeneralStatusCode() == eipScanner::cip::GeneralStatusCodes::SUCCESS) {
                        return {};
                    }
                    else {
                        return std::unexpected{std::to_string(static_cast<int16_t>(response.getGeneralStatusCode()))};
                    }
                } catch (std::exception &e) {
                    return std::unexpected{e.what()};
                }
            }

            /**
            * @brief Reads an integer value from an R register on the robot controller.
            * @param register_index The index of the R register to read from.
            * @return An expected containing the integer value on success, or an error message on failure. Resource temporarily unavailable error might mean that timeout was exceeded. You can specify this timeout in the FANUCRobot constructor.
            */
            std::expected<int16_t, std::string> read_R_register(const uint8_t register_index) {
                ZoneScoped;
                if(!connected) {
                    return std::unexpected{"Not connected yet."};
                }
                try {
                    const eipScanner::cip::CipUsint service_id = 0xe;
                    const eipScanner::cip::EPath epath(0x6B, 0x1, register_index);

                    auto response = message_router->sendRequest(session_info, service_id, epath);

                    if (response.getGeneralStatusCode() == eipScanner::cip::GeneralStatusCodes::SUCCESS) {
                        eipScanner::utils::Buffer buffer(response.getData());
                        eipScanner::cip::CipInt result;
                        buffer >> result;
                        return result;
                    }
                    else {
                        return std::unexpected{std::to_string(static_cast<int16_t>(response.getGeneralStatusCode()))};
                    }
                } catch (std::exception &e) {
                    return std::unexpected{e.what()};
                }
            }

            /**
            * @brief Reads the current pose of the robot.
            *
            * Retrieves the robot's current position and orientation.
            * @return An expected containing the robot_pose structure on success, or an error message on failure. Resource temporarily unavailable error might mean that timeout was exceeded. You can specify this timeout in the FANUCRobot constructor.
            */
            std::expected<robot_pose, std::string> read_current_pose() {
                ZoneScoped;
                if(!connected) {
                    return std::unexpected{"Not connected yet."};
                }
                try {
                    const eipScanner::cip::CipUsint service_id = eipScanner::cip::ServiceCodes::GET_ATTRIBUTE_SINGLE;
                    const eipScanner::cip::EPath epath(0x7D, 0x01, 0x01);

                    auto response = message_router->sendRequest(session_info, service_id, epath);

                    if (response.getGeneralStatusCode() == eipScanner::cip::GeneralStatusCodes::SUCCESS) {
                        eipScanner::utils::Buffer buffer(response.getData());
                        return parse_pose_buffer(buffer.data());
                    }
                    else {
                        return std::unexpected{"Error: " + std::to_string(static_cast<int16_t>(response.getGeneralStatusCode()))};
                    }
                } catch (std::exception &e) {
                    return std::unexpected{"Exception:" + std::string{e.what()}};
                }
            }

            /**
            * @brief Reads a pose from a specified PR (Position Register) on the robot controller.
            * @param register_index The index of the PR register to read from.
            * @return An expected containing the robot_pose structure on success, or an error message on failure. Resource temporarily unavailable error might mean that timeout was exceeded. You can specify this timeout in the FANUCRobot constructor.
            */
            std::expected<robot_pose, std::string> read_PR_register(const uint8_t register_index) {
                ZoneScoped;
                if(!connected) {
                    return std::unexpected{"Not connected yet."};
                }
                try {
                    const eipScanner::cip::CipUsint service_id = eipScanner::cip::ServiceCodes::GET_ATTRIBUTE_SINGLE;
                    const eipScanner::cip::EPath epath(0x7B, 0x01, register_index);

                    auto response = message_router->sendRequest(session_info, service_id, epath);

                    if (response.getGeneralStatusCode() == eipScanner::cip::GeneralStatusCodes::SUCCESS) {
                        eipScanner::utils::Buffer buffer(response.getData());
                        return parse_pose_buffer(buffer.data());
                    }
                    else {
                        return std::unexpected{"Error: " + std::to_string(static_cast<int16_t>(response.getGeneralStatusCode()))};
                    }
                } catch (std::exception &e) {
                    return std::unexpected{"Exception:" + std::string{e.what()}};
                }
            }

            /**
            * @brief Writes a pose to a specified PR (Position Register) on the robot controller.
            * @param register_index The index of the PR register to write to.
            * @param desired_pose The robot_pose structure containing the desired pose.
            * @return An expected containing void on success, or an error message on failure. Resource temporarily unavailable error might mean that timeout was exceeded. You can specify this timeout in the FANUCRobot constructor.
            */
            std::expected<void, std::string> write_PR_register(const uint8_t register_index, const fanuc_ethernet::robot_pose &desired_pose) {
                ZoneScoped;
                if(!connected) {
                    return std::unexpected{"Not connected yet."};
                }
                try {
                    const eipScanner::cip::CipUsint service_id = 0x10;
                    const eipScanner::cip::EPath epath(0x7B, 0x01, register_index);
                    eipScanner::utils::Buffer data;
                    // Apparently UFRAME and UTOOL have to always be 0.
                    data << (short)0 << (short)0 << desired_pose.x << desired_pose.y << desired_pose.z << desired_pose.yaw << desired_pose.pitch << desired_pose.roll
                        << desired_pose.turn1 << desired_pose.turn2 << desired_pose.turn3 << desired_pose.bitflip << desired_pose.E0 << desired_pose.E1 << desired_pose.E2;

                    // std::cout << data.data().size()<<std::endl;

                    auto response = message_router->sendRequest(session_info, service_id, epath, data.data());

                    if (response.getGeneralStatusCode() == eipScanner::cip::GeneralStatusCodes::SUCCESS) {
                        return {};
                    }
                    else {
                        return std::unexpected{std::to_string(static_cast<int16_t>(response.getGeneralStatusCode()))};
                    }
                } catch (std::exception &e) {
                    return std::unexpected{e.what()};
                }
            }

            /**
            * @brief Enables the robot to allow movement, by setting the enable register to 1, allowing the robot to execute movements.
            * @return An expected containing void on success, or an error message on failure. Resource temporarily unavailable error might mean that timeout was exceeded. You can specify this timeout in the FANUCRobot constructor.
            */
            std::expected<void, std::string> enable_robot() {
                ZoneScoped;
                return write_R_register(ENABLE_REGISTER, 1);
            }

            /**
            * @brief Disables the robot to stop movement, by setting the enable register to 0, stopping the robot's movement.
            * @return An expected containing void on success, or an error message on failure. Resource temporarily unavailable error might mean that timeout was exceeded. You can specify this timeout in the FANUCRobot constructor.
            */
            std::expected<void, std::string> disable_robot() {
                ZoneScoped;
                return write_R_register(ENABLE_REGISTER, 0);
            }

            /**
            * @brief Checks if the robot is currently enabled. 
            * Currently, the same register is used for both enabled and moving, which is suboptimal.
            * To fix this, a second register (moving_register) would have to be set when the movement is finished and the main loop would have to use the enable_register.
            * @return An expected containing a boolean value (true if enabled, false otherwise) on success, or an error message on failure. Resource temporarily unavailable error might mean that timeout was exceeded. You can specify this timeout in the FANUCRobot constructor.
            */
            std::expected<bool, std::string> is_enabled() {
                ZoneScoped;
                auto res = read_R_register(ENABLE_REGISTER);
                if(res.has_value()) {
                    return res.value() == 1;
                }
                else return std::unexpected {res.error()};
            }

            /**
            * @brief Sets movement mode to FINE and enables high speed skip. This should be used for final adjustments as it's limited to 100mm/s.
            * If controlling FINE/CNT and high speed skip separately is desired, the register could be used to store values other than 0 and 1.
            * However, the TP program needs to be adjusted accordingly in that case, of course.
            * This mode is suitable for precise movements but is limited in speed.
            * @return An expected containing void on success, or an error message on failure. Resource temporarily unavailable error might mean that timeout was exceeded. You can specify this timeout in the FANUCRobot constructor.
            */
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

            /**
            * @brief Sets the movement mode to CNT (continuous) and switches to "normal speed" skip.
            * This mode allows smoother movements suitable for general operations.
            * If controlling FINE/CNT and high speed skip separately is desired, the register could be used to store values other than 0 and 1.
            * However, the TP program needs to be adjusted accordingly in that case, of course.
            * @return An expected containing void on success, or an error message on failure. Resource temporarily unavailable error might mean that timeout was exceeded. You can specify this timeout in the FANUCRobot constructor.
            */
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

            /**
            * @brief Checks if the FINE high-speed skip mode is currently enabled.
            * @return True if FINE high-speed skip mode is enabled, false otherwise.
            */
            bool is_fine_high_speed_skip_enabled() {
                return fine_high_speed_skip_enabled;
            }

            /**
            * @brief Sets the velocity limit for robot movements.
            * @param vel_limit The velocity limit in millimeters per second.
            * @return An expected containing void on success, or an error message on failure. Resource temporarily unavailable error might mean that timeout was exceeded. You can specify this timeout in the FANUCRobot constructor.
            */
            std::expected<void, std::string> set_velocity_limit(const uint32_t vel_limit) {
                ZoneScoped;
                return write_R_register(VELOCITY_LIMIT_REGISTER, vel_limit);
            }

            /**
            * @brief Moves the robot to the specified pose synchronously. This function blocks until the movement is completed.
            * 
            * WARNING: This function is potentially dangerous as the robot will not stop until it reaches the position.
            * No force limits are checked during movement. Implement your own force checks on a separate thread if necessary.
            * @param desired_pose The target robot_pose to move to.
            * @return An expected containing void on success, or an error message on failure. Resource temporarily unavailable error might mean that timeout was exceeded. You can specify this timeout in the FANUCRobot constructor.
            */
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

            /**
            * @brief Moves the robot to the specified pose asynchronously. This function initiates the movement and returns immediately.
            * 
            * WARNING: This function is potentially dangerous as the robot will not stop until it reaches the position.
            * No force limits are checked during movement. Implement your own force checks on a separate thread if necessary.
            * @param desired_pose The target robot_pose to move to.
            * @param stop_current_movement If true, stops the current movement before initiating the new one.
            * @return An expected containing void on success, or an error message on failure. Resource temporarily unavailable error might mean that timeout was exceeded. You can specify this timeout in the FANUCRobot constructor.
            */
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

        private:
            /**
            * @brief Parses a float value from a buffer at a given offset.
            * @param buf The buffer containing the data.
            * @param offset The offset in the buffer where the float value starts.
            * @return An expected containing the float value on success, or an error message on failure.
            */
            std::expected<float, std::string> parse_float_from_buffer(const std::vector<uint8_t> &buf, const int16_t offset) {
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

            /**
            * @brief Parses a byte (uint8_t) value from a buffer at a given offset.
            * @param buf The buffer containing the data.
            * @param offset The offset in the buffer where the byte value starts.
            * @return An expected containing the byte value on success, or an error message on failure.
            */
            std::expected<uint8_t, std::string> parse_byte_from_buffer(const std::vector<uint8_t> &buf, const int16_t offset) {
                ZoneScoped;
                if(buf.size() < offset + sizeof(uint8_t)) {
                    return std::unexpected{"Buffer too small to parse byte"};
                }

                uint8_t temp;
                std::memcpy(&temp, &buf[offset], sizeof(temp));
                return temp;
            }

            /**
            * @brief Parses a robot_pose structure from a buffer.
            * @param buf The buffer containing the pose data.
            * @return An expected containing the robot_pose structure on success, or an error message on failure.
            */
            std::expected<robot_pose, std::string> parse_pose_buffer(const std::vector<uint8_t> &buf) {
                ZoneScoped;
                robot_pose pose {};

                pose.utool = buf[0] + buf[1] * 8;
                pose.uframe = buf[2] + buf[3] * 8;
                
                int16_t offset {4};

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
    };
}

#endif
