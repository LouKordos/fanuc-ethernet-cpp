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
    class FANUCRobot {
        private:
            std::string ip;
            std::shared_ptr<MessageRouter> message_router;
            std::shared_ptr<SessionInfo> session_info;
            bool connected {false};
        public:
            FANUCRobot(const std::string &ip) : ip(ip) {
                
            }

            bool initialize_connection() {
                ZoneScoped;
                // Open EIP session with the adapter, 0xAF12 is default CIP/EIP port
                session_info = std::make_shared<SessionInfo>(ip, 0xAF12);
                message_router = std::make_shared<MessageRouter>();
                connected = true;
                return true;
            }

            bool write_R_register(uint register_index, int32_t value) {
                ZoneScoped;
                if(!connected) {
                    return false;
                }
                try {
                    const CipUsint service_id = 0x10;
                    const EPath epath(0x6B, 0x1, register_index);
                    Buffer data;
                    data << value;

                    auto response = message_router->sendRequest(session_info, service_id, epath, data.data());

                    if (response.getGeneralStatusCode() == GeneralStatusCodes::SUCCESS) {
                        return true;
                    }
                    else {
                        std::cout << "Error: " << response.getGeneralStatusCode() << std::endl;
                        return false;
                    }
                } catch (std::exception &e) {
                    std::cout << "Exception:" << e.what() << std::endl;
                    return false;
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
                        std::cout << "Error: " << response.getGeneralStatusCode() << std::endl;
                        return std::unexpected{std::to_string(static_cast<int>(response.getGeneralStatusCode()))};
                    }
                } catch (std::exception &e) {
                    std::cout << "Exception:" << e.what() << std::endl;
                    return std::unexpected{e.what()};
                }
            }
    };
}

#endif