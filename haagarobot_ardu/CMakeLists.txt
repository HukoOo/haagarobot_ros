cmake_minimum_required(VERSION 2.8.3)
project(haagarobot_ardu)


find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES HaagaRobot_ardu
#  CATKIN_DEPENDS nav_msgs roscpp rospy std_msgs tf
#  DEPENDS system_lib
)


#***********************
#******   OpenCV
#***********************
find_package( OpenCV 3.1.0 REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(odom_broadcaster src/odom_broadcaster/main.cpp src/odom_broadcaster/main.h)
target_link_libraries(odom_broadcaster
   ${catkin_LIBRARIES}
)

