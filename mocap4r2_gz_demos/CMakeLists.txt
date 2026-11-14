cmake_minimum_required(VERSION 3.10)
project(mocap4r2_gz_demos)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds launch config rviz DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
