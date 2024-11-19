#ifndef FANUC_ETHERNET_CPP
#define FANUC_ETHERNET_CPP

#include <string>

// TODO: Include Tracy and measure send and connect times in detail, add message text to profiler metadata

namespace fanuc_ethernet {
    class FANUCRobot {
        private:
            std::string ip;
        public:
            FANUCRobot(const std::string &ip) : ip(ip) {
                
            }

            
    };
}

#endif