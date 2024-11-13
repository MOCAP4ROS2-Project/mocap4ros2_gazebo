cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

project(gz_mocap4r2_plugin)

find_package(ament_cmake REQUIRED)
find_package(gz-cmake3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mocap_interfaces REQUIRED)
find_package(mocap4r2_control REQUIRED)
find_package(rclcpp REQUIRED)

gz_find_package(gz-plugin2 REQUIRED COMPONENTS register)
gz_find_package(gz-sim8 REQUIRED)
gz_find_package(gz-rendering8 REQUIRED)
gz_find_package(gz-sensors8 REQUIRED)

set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})
set(GZ_RENDERING_VER ${gz-rendering8_VERSION_MAJOR})
set(GZ_SENSORS_VER ${gz-sensors8_VERSION_MAJOR})

include_directories(include)

add_library(gz_ros_mocap SHARED src/gz_ros_mocap.cpp)

ament_target_dependencies(gz_ros_mocap
  rclcpp
  geometry_msgs
  mocap_interfaces
  mocap4r2_control
)

target_link_libraries(gz_ros_mocap
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
)

install(DIRECTORY
  launch urdf
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS gz_ros_mocap DESTINATION lib/gz_ros_mocap)

ament_export_libraries(gz_ros_mocap)
ament_export_dependencies(gz-cmake3 gz-plugin2 gz-sim8 geometry_msgs mocap_interfaces mocap4r2_control rclcpp)

ament_package()
