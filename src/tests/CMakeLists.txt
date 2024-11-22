cmake_minimum_required(VERSION 3.28.0)

add_executable(move_test move_test.cpp)
target_link_libraries(move_test PRIVATE fanuc_ethernet)