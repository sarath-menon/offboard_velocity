cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(offboard)

if(MINGW OR MSVC)
    add_definitions("-D_USE_MATH_DEFINES") # For M_PI
endif()

if(MSVC)
    add_definitions("-WX -W2")
else()
    add_definitions("-Wall -Wextra")
endif()

find_package(MAVSDK REQUIRED)

add_executable(offboard_control offboard_velocity_working.cpp)
add_executable(offboard_read offboard_read_blocking.cpp)
add_executable(offboard_read_attitude offboard_read_attitude.cpp)
add_executable(offboard_health offboard_telemetry_health.cpp)
add_executable(offboard_vision offboard_vision.cpp)

target_link_libraries(offboard_read
    MAVSDK::mavsdk_action
    MAVSDK::mavsdk_offboard
    MAVSDK::mavsdk_telemetry
    MAVSDK::mavsdk_mocap
    MAVSDK::mavsdk
)

target_link_libraries(offboard_health
    MAVSDK::mavsdk_action
    MAVSDK::mavsdk_offboard
    MAVSDK::mavsdk_telemetry
    MAVSDK::mavsdk_mocap
    MAVSDK::mavsdk
)

target_link_libraries(offboard_control
    MAVSDK::mavsdk_action
    MAVSDK::mavsdk_offboard
    MAVSDK::mavsdk_telemetry
    MAVSDK::mavsdk_mocap
    MAVSDK::mavsdk
)

target_link_libraries(offboard_vision
    MAVSDK::mavsdk_action
    MAVSDK::mavsdk_offboard
    MAVSDK::mavsdk_telemetry
    MAVSDK::mavsdk_mocap
    MAVSDK::mavsdk
)

target_link_libraries(offboard_read_attitude
    MAVSDK::mavsdk_action
    MAVSDK::mavsdk_offboard
    MAVSDK::mavsdk_telemetry
    MAVSDK::mavsdk_mocap
    MAVSDK::mavsdk
)