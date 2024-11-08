cmake_minimum_required(VERSION 3.28.0)
add_executable(move_test move_test.cpp)
target_include_directories(move_test PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(move_test PUBLIC TracyClient)