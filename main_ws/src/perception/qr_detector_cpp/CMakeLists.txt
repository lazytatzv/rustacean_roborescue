cmake_minimum_required(VERSION 3.8)
project(qr_detector_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Build the component library
add_library(qr_detector_component SHARED
  src/qr_detector_node.cpp
)
target_link_libraries(qr_detector_component
  ${OpenCV_LIBS}
)
ament_target_dependencies(qr_detector_component
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  cv_bridge
  ament_index_cpp
)
rclcpp_components_register_node(qr_detector_component
  PLUGIN "qr_detector_cpp::QrDetectorNode"
  EXECUTABLE qr_detector_node
)

# Standalone executable (uses the component)
add_executable(qr_detector_node_exe src/main.cpp)
target_link_libraries(qr_detector_node_exe qr_detector_component)
ament_target_dependencies(qr_detector_node_exe rclcpp)

install(TARGETS
  qr_detector_component
  qr_detector_node_exe
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
