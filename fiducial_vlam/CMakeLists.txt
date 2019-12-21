cmake_minimum_required(VERSION 3.5)
project(fiducial_vlam)

#=============
# Setup
#=============

# Default to C99
if (NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif ()

# Emulate colcon
if ($ENV{CLION_IDE})
  message(STATUS "Running inside CLion")
  set(fiducial_vlam_msgs_DIR "${PROJECT_SOURCE_DIR}/../../../install/fiducial_vlam_msgs/share/fiducial_vlam_msgs/cmake")
  set(ros2_shared_DIR "${PROJECT_SOURCE_DIR}/../../../install/ros2_shared/share/ros2_shared/cmake")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DRUN_INSIDE_CLION")
endif ()

# Try for OpenCV 4.X, but settle for whatever is installed
find_package(OpenCV 4 QUIET)
if (NOT OpenCV_FOUND)
  find_package(OpenCV REQUIRED)
endif ()
message(STATUS "Found OpenCV version ${OpenCV_VERSION}")

# Find packages
find_package(ament_cmake REQUIRED)
find_package(class_loader REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(fiducial_vlam_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ros2_shared REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

# Local includes
include_directories(
  include
)

# Debugging: set _dump_all_variables to true
set(_dump_all_variables false)
if (_dump_all_variables)
  get_cmake_property(_variable_names VARIABLES)
  list(SORT _variable_names)
  foreach (_variable_name ${_variable_names})
    message(STATUS "${_variable_name}=${${_variable_name}}")
  endforeach ()
endif ()

# Create ament index resource which references the libraries in the binary dir
set(node_plugins "")

#=============
# vloc node
#=============

set(VLOC_NODE_SOURCES
  src/convert_util.cpp
  src/fiducial_math.cpp
  src/map.cpp
  src/transform_with_covariance.cpp
  src/vloc_context.cpp
  src/vloc_node.cpp
  )

set(VLOC_NODE_DEPS
  class_loader
  cv_bridge
  fiducial_vlam_msgs
  nav_msgs
  OpenCV
  rclcpp
  ros2_shared
  sensor_msgs
  std_msgs
  tf2_msgs
  )

add_library(vloc_node SHARED
  ${VLOC_NODE_SOURCES})

target_compile_definitions(vloc_node
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(vloc_node
  ${VLOC_NODE_DEPS})

rclcpp_components_register_nodes(vloc_node "fiducial_vlam::VlocNode")
set(node_plugins "${node_plugins}fiducial_vlam::VlocNode;$<TARGET_FILE:vloc_node>\n")

add_executable(vloc_main
  src/vloc_main.cpp
  ${VLOC_NODE_SOURCES})

ament_target_dependencies(vloc_main
  ${VLOC_NODE_DEPS})


#=============
# vmap node
#=============

set(VMAP_NODE_SOURCES
  src/convert_util.cpp
  src/fiducial_math.cpp
  src/map.cpp
  src/transform_with_covariance.cpp
  src/vmap_context.cpp
  src/vmap_main.cpp
  src/vmap_node.cpp
  )

set(VMAP_NODE_DEPS
  class_loader
  fiducial_vlam_msgs
  geometry_msgs
  OpenCV
  rclcpp
  ros2_shared
  std_msgs
  tf2_msgs
  visualization_msgs
  yaml_cpp_vendor
  )

add_library(vmap_node SHARED
  ${VMAP_NODE_SOURCES})

target_compile_definitions(vmap_node
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(vmap_node
  ${VMAP_NODE_DEPS})

rclcpp_components_register_nodes(vmap_node "fiducial_vlam::VmapNode")
set(node_plugins "${node_plugins}fiducial_vlam::VmapNode;$<TARGET_FILE:vmap_node>\n")

add_executable(vmap_main
  src/vmap_main.cpp
  ${VMAP_NODE_SOURCES})

ament_target_dependencies(vmap_main
  ${VMAP_NODE_DEPS})


#=============
# Install
#=============

# Install composable nodes
install(TARGETS
  vloc_node
  EXPORT export_vloc_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

install(TARGETS
  vmap_node
  EXPORT export_vmap_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

# Install executables
install(TARGETS
  vloc_main
  vmap_main
  DESTINATION lib/fiducial_vlam
  )

# Install various directories
install(DIRECTORY
  cfg
  launch
  DESTINATION share/fiducial_vlam
  )

install(DIRECTORY include/
  DESTINATION include)


#=============
# Export
#=============

ament_export_dependencies(class_loader yaml_cpp_vendor)

ament_export_include_directories(include)

ament_export_interfaces(export_vloc_node export_vmap_node)

ament_export_libraries(vloc_node vmap_node)


#=============
# Run ament macros
#=============

ament_package()
