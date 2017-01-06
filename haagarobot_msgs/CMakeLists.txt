cmake_minimum_required(VERSION 2.8.3)
project(haagarobot_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Motor.msg
   GPS.msg
   cmd_rpm.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
 #  Cmd_vel.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)



