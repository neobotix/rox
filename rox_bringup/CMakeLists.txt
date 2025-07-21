cmake_minimum_required(VERSION 3.0.2)
project(rox_bringup)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch configs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
