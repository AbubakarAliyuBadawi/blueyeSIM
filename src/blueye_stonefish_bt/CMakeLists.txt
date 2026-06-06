cmake_minimum_required(VERSION 3.8)
project(blueye_stonefish_bt)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mundus_mir_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

add_library(bt_nodes SHARED
  src/behaviors/navigate_to_waypoint.cpp
  src/behaviors/station_keeping.cpp
  src/behaviors/launch_docking_procedure.cpp
  src/behaviors/wait_node.cpp
  src/conditions/battery_condition.cpp
  src/conditions/camera_condition.cpp
  src/conditions/sonar_condition.cpp
  src/conditions/system_watchdog.cpp
  src/conditions/blackboard_condition.cpp
  src/actions/altitude_control_action.cpp
  src/actions/publish_state.cpp
)

ament_target_dependencies(bt_nodes
  rclcpp behaviortree_cpp mundus_mir_msgs
  nav_msgs std_msgs std_srvs sensor_msgs geometry_msgs
)

target_link_libraries(bt_nodes
  ${behaviortree_cpp_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

add_executable(blueye_stonefish_bt src/main.cpp)
target_link_libraries(blueye_stonefish_bt bt_nodes ${behaviortree_cpp_LIBRARIES} ${rclcpp_LIBRARIES})
ament_target_dependencies(blueye_stonefish_bt
  rclcpp behaviortree_cpp mundus_mir_msgs
  std_msgs nav_msgs geometry_msgs std_srvs sensor_msgs
)

install(TARGETS bt_nodes blueye_stonefish_bt
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY behavior_trees launch config DESTINATION share/${PROJECT_NAME})

ament_package()
