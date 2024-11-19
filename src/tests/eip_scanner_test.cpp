#include <EIPScanner/MessageRouter.h>
#include <EIPScanner/utils/Logger.h>
#include <EIPScanner/utils/Buffer.h>
#include <iostream>
#include <expected>
#include <string>

using eipScanner::SessionInfo;
using eipScanner::MessageRouter;
using namespace eipScanner::cip;
using namespace eipScanner::utils;

std::expected<int, std::string> read_r_register(const std::string &ip, uint register_index) {
    try {
        // Open EIP session with the adapter, 0xAF12 is default CIP/EIP port
        auto si = std::make_shared<SessionInfo>(ip, 0xAF12);
        MessageRouter messageRouter;

        const CipUsint service_id = 0xe;
        const EPath epath(0x6B, 0x1, register_index);

        auto response = messageRouter.sendRequest(si, service_id, epath);

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

bool write_r_register(const std::string &ip, uint register_index, int32_t value) {
    try {
        // Open EIP session with the adapter, 0xAF12 is default CIP/EIP port
        auto si = std::make_shared<SessionInfo>(ip, 0xAF12);
        MessageRouter messageRouter;

        const CipUsint service_id = 0x10;
        const EPath epath(0x6B, 0x1, register_index);

        Buffer data;
        data << value;

        auto response = messageRouter.sendRequest(si, service_id, epath, data.data());

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

int main() {
    for(int i = 0; i < 10; i++) {
int value = read_r_register("10.36.12.3", 1).value();
    std::cout << "Value before: " << value << std::endl;

    write_r_register("10.36.12.3", 1, value+1);

    value = read_r_register("10.36.12.3", 1).value();
    std::cout << "Value after: " << value << std::endl;
    }
    
    return 0;
}
