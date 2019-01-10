cmake_minimum_required(VERSION 3.5)
project(flock_vlam)

#=============
# Setup
#=============

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Emulate colcon
if($ENV{CLION_IDE})
  message("Running inside CLion")
  set(flock_vlam_msgs_DIR "${PROJECT_SOURCE_DIR}/../../../install/flock_vlam_msgs/share/flock_vlam_msgs/cmake")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DRUN_INSIDE_CLION")
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(flock_vlam_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

# Local includes
include_directories(
  include
  ${flock_vlam_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

#=============
# vloc node
#=============

add_executable(vloc_node
  src/vloc_node.cpp
  src/map.cpp
  src/tf2_util.cpp)

ament_target_dependencies(vloc_node
  cv_bridge
  flock_vlam_msgs
  geometry_msgs
  OpenCV
  rclcpp
  sensor_msgs
  std_msgs
  )

#=============
# vmap node
#=============

add_executable(vmap_node
  src/vmap_node.cpp
  src/map.cpp
  src/tf2_util.cpp)

ament_target_dependencies(vmap_node
  cv_bridge
  flock_vlam_msgs
  geometry_msgs
  OpenCV
  rclcpp
  sensor_msgs
  std_msgs
  yaml_cpp_vendor
  )

#=============
# Install
#=============

# Install targets
install(
  TARGETS
  vloc_node
  vmap_node
  DESTINATION lib/flock_vlam
)

# Install various directories
install(
  DIRECTORY
  cfg
  launch
  DESTINATION share/flock_vlam
)

#=============
# Run ament macros
#=============

ament_package()
