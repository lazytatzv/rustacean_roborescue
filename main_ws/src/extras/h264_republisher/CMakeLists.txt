cmake_minimum_required(VERSION 3.10)
project(h264_republisher)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(LIBAVFORMAT REQUIRED libavformat)
pkg_check_modules(LIBAVCODEC REQUIRED libavcodec)
pkg_check_modules(LIBSWSCALE REQUIRED libswscale)
pkg_check_modules(LIBAVUTIL REQUIRED libavutil)

include_directories(
  ${LIBAVFORMAT_INCLUDE_DIRS}
  ${LIBAVCODEC_INCLUDE_DIRS}
  ${LIBSWSCALE_INCLUDE_DIRS}
  ${LIBAVUTIL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
)

add_executable(h264_republisher src/h264_republisher.cpp)

target_include_directories(h264_republisher PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(h264_republisher
  ${LIBAVFORMAT_LIBRARIES}
  ${LIBAVCODEC_LIBRARIES}
  ${LIBSWSCALE_LIBRARIES}
  ${LIBAVUTIL_LIBRARIES}
  ${OpenCV_LIBS}
)

ament_target_dependencies(h264_republisher
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  image_transport
)

install(TARGETS h264_republisher
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
