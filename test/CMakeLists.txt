find_package(ament_cmake_gtest REQUIRED)

option(ENABLE_DISPLAY_TESTS "Enable the building of tests that requires a display" OFF)


# Tests
set(tests
  test_gazebo_ros_mocap
)

foreach(test ${tests})
  ament_add_gtest(${test}
    ${test}.cpp
    WORKING_DIRECTORY
      ${CMAKE_CURRENT_BINARY_DIR}
    # Long timeout because has to run gazebo several times
    TIMEOUT
      120
  )
  target_link_libraries(${test}
    gazebo_test_fixture
  )
  ament_target_dependencies(${test}
    "gazebo_dev"
    "gazebo_msgs"
    "gazebo_ros"
    "geometry_msgs"
    "rclcpp"
  )
  if(ENABLE_PROFILER)
    target_include_directories(${test} PUBLIC ${ignition-common3_INCLUDE_DIRS})
    target_link_libraries(${test} ${ignition-common3_LIBRARIES})
  endif()
endforeach()
