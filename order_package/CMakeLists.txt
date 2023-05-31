cmake_minimum_required(VERSION 3.0.2)
project(order_package)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  geometry_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  OrderGoal.msg
  OrderRequest.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  AddTask.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#    DIRECTORY action 
#    FILES CustomerInteraction.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES look_for_item
    CATKIN_DEPENDS std_msgs message_runtime geometry_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
