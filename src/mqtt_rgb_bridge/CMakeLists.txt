cmake_minimum_required(VERSION 3.8)
project(mqtt_rgb_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Find RealSense installed from source
find_path(REALSENSE2_INCLUDE_DIR librealsense2/rs.hpp
  PATHS /usr/local/include /usr/include
  REQUIRED
)
find_library(REALSENSE2_LIBRARY realsense2
  PATHS /usr/local/lib /usr/lib/aarch64-linux-gnu /usr/lib
  REQUIRED
)

add_executable(rgb_publisher src/rgb_publisher.cpp)

target_include_directories(rgb_publisher PUBLIC
  ${REALSENSE2_INCLUDE_DIR}
)

ament_target_dependencies(rgb_publisher
  rclcpp
  sensor_msgs
  std_msgs
  OpenCV
)

target_link_libraries(rgb_publisher
  ${REALSENSE2_LIBRARY}
)

install(TARGETS rgb_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
