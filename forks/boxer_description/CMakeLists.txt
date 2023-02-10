cmake_minimum_required(VERSION 2.8.3)

project(boxer_description)

find_package(catkin REQUIRED)

catkin_package(LIBRARIES ${PROJECT_NAME})

install(
  DIRECTORY launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
