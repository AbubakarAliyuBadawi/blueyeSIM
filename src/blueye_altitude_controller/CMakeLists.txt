cmake_minimum_required(VERSION 3.8)
project(blueye_altitude_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(marine_acoustic_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

# Add the executable
add_executable(altitude_controller
  src/altitude_controller.cpp
  src/main.cpp
)

# Add dependencies to the executable
ament_target_dependencies(altitude_controller 
  rclcpp
  marine_acoustic_msgs
  nav_msgs
  geometry_msgs
  std_msgs
)

# Install the executable
install(TARGETS
  altitude_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install include directories
install(DIRECTORY
  include/
  DESTINATION include
)

ament_package()