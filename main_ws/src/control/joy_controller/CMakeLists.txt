cmake_minimum_required(VERSION 3.14)
project(joy_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs QUIET)
find_package(custom_interfaces REQUIRED)

add_executable(joy_controller_node src/joy_controller.cpp)
ament_target_dependencies(
  joy_controller_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  control_msgs
  custom_interfaces
)
if(moveit_msgs_FOUND)
  target_compile_definitions(joy_controller_node PRIVATE HAVE_MOVEIT_MSGS)
  ament_target_dependencies(joy_controller_node moveit_msgs)
endif()

install(TARGETS
  joy_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
