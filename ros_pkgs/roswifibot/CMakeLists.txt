cmake_minimum_required(VERSION 2.8.3)
project(roswifibot)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
  rospy
  nav_msgs
  tf
  geometry_msgs
  sensor_msgs
  actionlib_msgs
  message_filters
  filters
  actionlib
  tf_conversions
  robot_state_publisher
  message_generation
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  speed_msg.msg
  vn100_msg.msg
  VnMatrix3x3.msg
  VnQuaternion_msg.msg
  VnVector3_msg.msg
  WStatus.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES libwifibot
  CATKIN_DEPENDS message_runtime std_msgs roscpp rospy nav_msgs tf geometry_msgs sensor_msgs actionlib_msgs message_filters filters actionlib tf_conversions robot_state_publisher
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(libwifibot
  src/libwifibot.cpp
)
add_library(vn100
  src/vn100.c
)
add_library(vncp_services
  src/vncp_services.c
)

## Declare a cpp executable
add_executable(vectornav_node src/vectornav.cpp)
add_dependencies(vectornav_node ${PROJECT_NAME}_generate_messages_cpp)

add_executable(listenerVn src/listenerVn.cpp)
add_dependencies(listenerVn ${PROJECT_NAME}_generate_messages_cpp)

add_executable(wifibot_node src/wifibot.cpp)
add_dependencies(wifibot_node ${PROJECT_NAME}_generate_messages_cpp)

#add_executable(listener src/listenerodo.cpp)

add_executable(wifibotControl_node src/wifibotControl.cpp)
add_dependencies(wifibotControl_node ${PROJECT_NAME}_generate_messages_cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(vectornav_node ${catkin_LIBRARIES})
target_link_libraries(vectornav_node vn100)
target_link_libraries(vectornav_node vncp_services)

target_link_libraries(listenerVn ${catkin_LIBRARIES})
target_link_libraries(listenerVn vn100)
target_link_libraries(listenerVn vncp_services)

#target_link_libraries(listener ${catkin_LIBRARIES})
#target_link_libraries(listener libwifibot)

target_link_libraries(wifibot_node ${catkin_LIBRARIES})
target_link_libraries(wifibot_node libwifibot)

#link libraries for Wifibot Control Node
target_link_libraries(wifibotControl_node ${catkin_LIBRARIES})
target_link_libraries(wifibotControl_node libwifibot)
target_link_libraries(wifibotControl_node vn100)
target_link_libraries(wifibotControl_node vncp_services)



#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS nao_smach_utils nao_smach_utils_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_nao_smach_utils.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)



#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#link libraries


