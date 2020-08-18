cmake_minimum_required(VERSION 3.0.2)
project(ur_planning)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    interactive_markers
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ur_pick_place_demo src/ur_pick_place_demo.cpp)
target_link_libraries(ur_pick_place_demo ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS ur_pick_place_demo DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
add_executable(cpp_interface src/cpp_interface.cpp)
target_link_libraries(cpp_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS cpp_interface DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
