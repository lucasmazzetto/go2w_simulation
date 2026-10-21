cmake_minimum_required(VERSION 3.8)
project(go2w_gazebo)

# Build dependencies for the C++ nodes and shared processing code.
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(unitree_go REQUIRED)

# Share sensor and telemetry processing between the nodes and their unit tests.
# PUBLIC requirements also apply to targets that link this library.
add_library(go2w_gazebo_processing STATIC
  src/joint_feedback.cpp
  src/utils.cpp)
target_compile_features(go2w_gazebo_processing PUBLIC cxx_std_17)
target_compile_options(go2w_gazebo_processing PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(go2w_gazebo_processing PUBLIC include)
ament_target_dependencies(go2w_gazebo_processing rclcpp sensor_msgs unitree_go)

# ROS executables use common compiler settings and declare dependencies per target.
add_executable(cmd_vel_adapter src/cmd_vel_adapter_node.cpp)
add_executable(sensor_adapter src/sensor_adapter_node.cpp)
add_executable(lowstate_publisher src/lowstate_publisher_node.cpp)

foreach(target cmd_vel_adapter sensor_adapter lowstate_publisher)
  target_compile_features(${target} PUBLIC cxx_std_17)
  target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
  target_include_directories(${target} PRIVATE include)
endforeach()

ament_target_dependencies(cmd_vel_adapter rclcpp geometry_msgs)
ament_target_dependencies(sensor_adapter builtin_interfaces rclcpp sensor_msgs unitree_go)
target_link_libraries(sensor_adapter go2w_gazebo_processing)
ament_target_dependencies(lowstate_publisher rclcpp sensor_msgs unitree_go)
target_link_libraries(lowstate_publisher go2w_gazebo_processing)

# Install the processing library and the executables used by launch and ros2 run.
install(TARGETS go2w_gazebo_processing
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

install(TARGETS cmd_vel_adapter sensor_adapter lowstate_publisher DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

# Keep manual integration checks available even when BUILD_TESTING is disabled.
install(PROGRAMS test/smoke_simulation.py test/smoke_sensors.py
  test/smoke_camera.py test/check_camera_startup.py test/check_lowstate.py
  test/check_lowstate_startup.py
  DESTINATION lib/${PROJECT_NAME})
# Install the helper beside the scripts for sibling imports; it is not executable.
install(FILES test/test_utils.py DESTINATION lib/${PROJECT_NAME})

# Preserve relative asset paths under the package share directory.
# Gazebo search paths are configured by sim.launch.py.
install(DIRECTORY config launch rviz urdf worlds DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(FILES README.md LICENSE DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  # Register Python model, configuration, launch, and helper tests with colcon test.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(model test/test_model.py)
  ament_add_pytest_test(sensors test/test_sensors.py)
  ament_add_pytest_test(test_utilities test/test_shared_utils.py)
  ament_add_pytest_test(spawn test/test_spawn.py)

  # Exercise the production C++ processing functions directly with GoogleTest.
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(lowstate_processing test/test_lowstate_processing.cpp)
  ament_target_dependencies(lowstate_processing rclcpp sensor_msgs unitree_go)
  ament_add_gtest(sensor_processing test/test_sensor_processing.cpp)
  ament_target_dependencies(sensor_processing sensor_msgs unitree_go)
  ament_add_gtest(camera_validation test/test_camera_validation.cpp)
  ament_target_dependencies(camera_validation sensor_msgs)

  # Reuse the processing library in every C++ test target.
  foreach(target lowstate_processing sensor_processing camera_validation)
    target_compile_features(${target} PUBLIC cxx_std_17)
    target_include_directories(${target} PRIVATE include)
    target_link_libraries(${target} go2w_gazebo_processing)
  endforeach()
endif()

# Generate ament package metadata after registering the install and test rules.
ament_package()
