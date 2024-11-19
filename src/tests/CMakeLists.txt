cmake_minimum_required(VERSION 3.28.0)
add_executable(move_test move_test.cpp)
target_include_directories(move_test PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(move_test PUBLIC TracyClient)

add_executable(eip_scanner_test eip_scanner_test.cpp)
include_directories("/usr/local/include/EIPScanner")
target_link_libraries(eip_scanner_test EIPScanner)
