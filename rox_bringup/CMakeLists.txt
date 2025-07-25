cmake_minimum_required(VERSION 3.5)

project(rox_bringup)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Save Waypoints Server Executable
add_executable(save_waypoints_server src/save_waypoints_server.cpp)
ament_target_dependencies(save_waypoints_server
  rclcpp visualization_msgs std_srvs)
target_link_libraries(save_waypoints_server yaml-cpp)

install(TARGETS save_waypoints_server
  DESTINATION lib/${PROJECT_NAME})

# Waypoint Looper Executable
add_executable(waypoint_looper src/waypoint_looper.cpp)
ament_target_dependencies(waypoint_looper
  rclcpp rclcpp_action nav2_msgs geometry_msgs std_srvs)
target_link_libraries(waypoint_looper yaml-cpp)

install(TARGETS waypoint_looper
  DESTINATION lib/${PROJECT_NAME})

# Install launch files and configs
install(DIRECTORY launch configs
  DESTINATION share/${PROJECT_NAME})

ament_package()
