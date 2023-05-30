cmake_minimum_required(VERSION 3.0.2)
project(voice_requests)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  actionlib_msgs
  actionlib
  message_generation
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
add_action_files(
   DIRECTORY action 
   FILES CustomerInteraction.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   actionlib_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES look_for_item
    CATKIN_DEPENDS actionlib_msgs std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
