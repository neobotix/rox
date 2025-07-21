cmake_minimum_required(VERSION 3.0.2)
project(rox_navigation)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch configs maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
