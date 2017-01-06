cmake_minimum_required(VERSION 2.8.3)
project(haagarobot_navigation)


find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  cv_bridge
  image_transport
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES HaagaRobot_navigation
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)


#***********************
#******   OpenCV
#***********************
find_package( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(lineFollower src/lineFollower/main.cpp src/lineFollower/main.h)
target_link_libraries(lineFollower
	${OpenCV_LIBS}
	${catkin_LIBRARIES}
)
add_executable(TopviewNode src/TopviewNode/main.cpp src/TopviewNode/main.h)
target_link_libraries(TopviewNode
	${OpenCV_LIBS}
	${catkin_LIBRARIES}
)
