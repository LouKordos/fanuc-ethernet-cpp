set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -save-temps -fverbose-asm -Wall -Wextra -march=native -mtune=native")
set(CMAKE_CXX_FLAGS_DEBUG "-g3 -ffold-simple-inlines")
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

set(default_build_type "Release")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
      STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release")
endif()

include(FetchContent)

set(EIPSCANNER_BUILD_SHARED ON CACHE BOOL "" FORCE)
FetchContent_Declare(
  eip_scanner
  GIT_REPOSITORY https://github.com/nimbuscontrols/EIPScanner.git
  GIT_TAG d20ef613e321da71a8297a813be2c330627e9b34
  GIT_PROGRESS TRUE
)

FetchContent_Declare (
  tracy
  GIT_REPOSITORY https://github.com/wolfpld/tracy.git
  GIT_TAG 759b4c3bfe207ab382c01080f0417aca5af4411a
  GIT_PROGRESS TRUE
)

FetchContent_MakeAvailable(tracy)

FetchContent_MakeAvailable(eip_scanner)
# Manually set the include directories for EIPScanner
target_include_directories(EIPScanner PUBLIC ${eip_scanner_SOURCE_DIR}/src)
get_target_property(EIPSCANNER_INCLUDE_DIRS EIPScanner INTERFACE_INCLUDE_DIRECTORIES) 
message(STATUS "EIPScanner include directories: ${EIPSCANNER_INCLUDE_DIRS}")


option(TRACY_ENABLE "" ON)

add_library(fanuc_ethernet INTERFACE)
target_include_directories(fanuc_ethernet INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(fanuc_ethernet INTERFACE TracyClient EIPScanner)


# Link Time Optimization
include(CheckIPOSupported)
check_ipo_supported(RESULT result)
if(result)
  set_target_properties(fanuc_ethernet PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

add_subdirectory(tests)
