cmake_minimum_required(VERSION 2.8.3)

project(boxer_msgs)

find_package(catkin REQUIRED)

catkin_package(LIBRARIES ${PROJECT_NAME})

install(
  DIRECTORY msg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
