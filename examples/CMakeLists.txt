cmake_minimum_required(VERSION 2.8.3)
project(examples)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
#gdb debug
#set (CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS}  -g ")
#set (CMAKE_VERBOSE_MAKEFILE ON)

find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
#find_package(yeebot_commute)

find_package(catkin REQUIRED
  COMPONENTS
    
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_planners_ompl
    pluginlib
    geometric_shapes
    moveit_visual_tools
    yeebot_core
)

find_package(Boost REQUIRED system filesystem date_time thread)
catkin_package(
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
  DEPENDS
    EIGEN3
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  "include/ur5_planning"
  "/home/yee/ros_ws/yeebot/devel/include"  #change
)
include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(max_length_ur5 src/max_length_ur5.cpp)
target_link_libraries(max_length_ur5 ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS max_length_ur5 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(sda5f_compare src/sda5f_compare.cpp)
target_link_libraries(sda5f_compare ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS sda5f_compare DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(dual_arm src/dual_arm.cpp)
target_link_libraries(dual_arm ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS dual_arm DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(dual_arm_api_test src/dual_arm_api_test.cpp)
target_link_libraries(dual_arm_api_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS dual_arm_api_test DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


