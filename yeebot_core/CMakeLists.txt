cmake_minimum_required(VERSION 2.8.3)
project(yeebot_core)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

#debug
#set (CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS}  -g ")
#set (CMAKE_VERBOSE_MAKEFILE ON)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  

  urdf
  srdfdom
  pluginlib
  kdl_parser
  #planning context depends
  moveit_core
  geometric_shapes
  moveit_ros_planning_interface
  moveit_simple_controller_manager
  moveit_planners_ompl
  trac_ik_lib
)  

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(orocos_kdl REQUIRED)
find_package(ompl REQUIRED)
#find_package(trac_ik_lib REQUIRED)

#set(ompl_include_dirs /home/yee/ompl_moveit/devel/include/ompl
#)
#set(ompl_libraries /home/yee/ompl_moveit/build/ompl/libompl.so
#)
catkin_package(
  CATKIN_DEPENDS
    roscpp
    urdf
    srdfdom
    pluginlib
    kdl_parser  
    moveit_core 
    #moveit_planners_ompl
    moveit_ros_planning_interface
    moveit_simple_controller_manager
    rviz_visual_tools
    #trac_ik_lib
                       #important
  
  LIBRARIES 
    ${PROJECT_NAME} 
    ${orocos_kdl_LIBRARIES}  
   # ${ompl_libraries}#important
  INCLUDE_DIRS  #which is neede to indicate the include path
    include
   # ${OMPL_INCLUDE_DIRS}
  DEPENDS
    EIGEN3
    orocos_kdl
    #trac_ik_lib
   # ompl
)

include_directories(
  INCLUDE_DIRS include
  ${catkin_INCLUDE_DIRS}
  SYSTEM ${EIGEN3_INCLUDE_DIRS}
  SYSTEM ${Boost_INCLUDE_DIRS}
  #SYSTEM ${orocos_kdl_INCLUDE_DIRS} #otherwise error// old version
  #SYSTEM ${OMPL_INCLUDE_DIRS}
  #SYSTEM ${trac_ik_lib_INCLUDE_DIRS}
)
#link_directories(BEFORE ${OMPL_LIBRARY_DIRS})




add_library(${PROJECT_NAME}
  src/kine_kdl.cpp
  src/pose_constraint.cpp
  src/planning_context.cpp
  src/iksolverpos_optp.cpp
  src/iksolverpos_tlp.cpp
  src/iksolverpos_trackp.cpp
  src/iksolvervel_project.cpp
  src/cbirrt.cpp
  src/planning_manager.cpp
)
target_link_libraries(
  ${PROJECT_NAME}
   ${catkin_LIBRARIES} 
   ${urdfdom_LIBRARIES} 
   ${urdfdom_headers_LIBRARIES}
  ${OMPL_LIBRARIES} 
  ${Boost_LIBRARIES} 
  #${orocos_kdl_LIBRARIES} 
  "/usr/lib/x86_64-linux-gnu/libnlopt.a")
#ompl should be OMPL

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
 )



