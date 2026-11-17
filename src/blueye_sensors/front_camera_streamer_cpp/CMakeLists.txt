cmake_minimum_required(VERSION 3.8)
project(front_camera_streamer_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(std_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Add executable
add_executable(front_camera_streamer src/camera_streamer_cpp.cpp)

# Add dependencies
ament_target_dependencies(front_camera_streamer
  rclcpp
  sensor_msgs
  cv_bridge
  std_msgs
  image_transport
  OpenCV
)

install(TARGETS
  front_camera_streamer
  DESTINATION lib/${PROJECT_NAME}
)



ament_package()
