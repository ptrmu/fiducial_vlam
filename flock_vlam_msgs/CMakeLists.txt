cmake_minimum_required(VERSION 3.5)
project(flock_vlam_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

# Emulate colcon in CLion
if($ENV{CLION_IDE})
    message("Running inside CLion")
    find_package(fastrtps_cmake_module REQUIRED)
    set(FastRTPS_INCLUDE_DIR "/opt/ros/bouncy/include")
    set(FastRTPS_LIBRARY_RELEASE "/opt/ros/bouncy/lib/libfastrtps.so")
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
        ${geometry_msgs_INCLUDE_DIRS}
        ${sensor_msgs_INCLUDE_DIRS}
        ${std_msgs_INCLUDE_DIRS}
)

set(msg_files
    "msg/Map.msg"
    "msg/Observation.msg"
    "msg/Observations.msg"
    )

# Generate ROS interfaces
rosidl_generate_interfaces(
    ${PROJECT_NAME}
    ${msg_files}
    DEPENDENCIES std_msgs geometry_msgs sensor_msgs
    )

ament_package()
