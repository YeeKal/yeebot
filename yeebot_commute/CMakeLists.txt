cmake_minimum_required(VERSION 2.8.3)
project(yeebot_commute)

add_compile_options(-std=c++11)

set (CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS}  -g ")
set (CMAKE_VERBOSE_MAKEFILE ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  message_generation
)

add_service_files(
  FILES
  JointInfo.srv
)

 generate_messages(
   DEPENDENCIES
   std_msgs
 )
catkin_package(
  CATKIN_DEPENDS
  LIBRARIES 
    ${PROJECT_NAME} 
  INCLUDE_DIRS  #which is neede to indicate the include path
    include
  DEPENDS
)




 include_directories(
  INCLUDE_DIRS include
  ${catkin_INCLUDE_DIRS}
 )




add_executable(get_joint_states src/get_joint_states.cpp )
target_link_libraries(get_joint_states ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS get_joint_states DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
