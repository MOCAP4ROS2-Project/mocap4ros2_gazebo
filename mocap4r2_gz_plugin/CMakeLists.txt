cmake_minimum_required(VERSION 3.10)
project(mocap4r2_gz_plugin)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(mocap4r2_control REQUIRED)
find_package(mocap4r2_interfaces REQUIRED)
find_package(mocap4r2_optitrack_driver REQUIRED)
find_package(rclcpp REQUIRED)

# Scene logic and NatNet bridge, without Gazebo (unit tested).
add_library(mocap4r2_gz_scene SHARED src/mocap_scene.cpp src/natnet_bridge.cpp)
target_include_directories(mocap4r2_gz_scene PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(mocap4r2_gz_scene PUBLIC
  mocap4r2_control::mocap4r2_control
  mocap4r2_optitrack_driver::natnet_protocol
)

# The Gazebo system plugin.
add_library(${PROJECT_NAME} SHARED src/mocap_system.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE
  mocap4r2_gz_scene
  gz-sim::gz-sim
  gz-plugin::register
  ${lifecycle_msgs_TARGETS}
  rclcpp::rclcpp
)

# Walking mannequin of the human motion demo (one Gazebo plugin per library).
add_library(mocap4r2_gz_gait_animator SHARED src/gait_animator.cpp)
target_link_libraries(mocap4r2_gz_gait_animator PRIVATE gz-sim::gz-sim gz-plugin::register)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY worlds launch DESTINATION share/${PROJECT_NAME})
install(TARGETS mocap4r2_gz_scene ${PROJECT_NAME} mocap4r2_gz_gait_animator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mocap_scene test/test_mocap_scene.cpp)
  target_link_libraries(test_mocap_scene mocap4r2_gz_scene mocap4r2_optitrack_driver::mocap4r2_optitrack_driver)
endif()

ament_package()
