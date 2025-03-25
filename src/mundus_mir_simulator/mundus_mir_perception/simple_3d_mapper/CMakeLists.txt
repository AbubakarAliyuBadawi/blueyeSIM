cmake_minimum_required(VERSION 3.8)
project(simple_3d_mapper)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(cv_bridge REQUIRED)

set(OCTOMAP_LIBRARIES "/opt/ros/humble/lib/x86_64-linux-gnu/liboctomap.so")

include_directories(
  include
  ${OCTOMAP_INCLUDE_DIRS}
)

add_executable(simple_3d_mapper src/main.cpp src/simple_3d_mapper.cpp)

ament_target_dependencies(simple_3d_mapper
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  octomap_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  octomap
  cv_bridge
)

target_link_libraries(simple_3d_mapper
  ${OCTOMAP_LIBRARIES}
)

install(TARGETS simple_3d_mapper
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()