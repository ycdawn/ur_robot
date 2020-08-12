cmake_minimum_required(VERSION 3.0.2)
project(ur_planning)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  moveit_msgs
  rospy
  std_msgs
  trajectory_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ur_planning
#  CATKIN_DEPENDS moveit_msgs rospy std_msgs trajectory_msgs
#  DEPENDS system_lib
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


