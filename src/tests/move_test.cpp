#include "../include/fanuc-ethernet.hpp"
#include <print>

int main() {
    fanuc_ethernet::FANUCRobot robot{"10.36.12.3"};
    std::println("Robot object initialized");
    return 0;
}