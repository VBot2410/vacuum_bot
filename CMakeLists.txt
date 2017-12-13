cmake_minimum_required(VERSION 2.8.3)
project(vacuum_bot)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  actionlib
  actionlib_msgs
  move_base_msgs
  std_msgs
  tf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )


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
  LIBRARIES main Goals Cleaner
  CATKIN_DEPENDS roscpp rospy actionlib actionlib_msgs geometry_msgs move_base_msgs std_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
   include
  ${catkin_INCLUDE_DIRS}
)

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

## Declare a C++ executable
 add_executable(Clean src/main.cpp src/Goals.cpp src/Cleaner.cpp)
## Specify libraries to link a library or executable target against
 target_link_libraries(Clean ${catkin_LIBRARIES})
## Adding class definitions
### add_library(vacuum_bot src/vacuum_bot.cpp)
### target_link_libraries(Clean vacuum_bot)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(
    vacuum_bot_test
    test/vacuum_bot_test.launch
    test/Cleaner_Test.cpp
    test/Goals_Test.cpp
    test/main.cpp
    src/Goals.cpp
    src/Cleaner.cpp
  )

  target_link_libraries(
    vacuum_bot_test
    ${catkin_LIBRARIES}
  )

  add_dependencies(
    vacuum_bot_test
    Clean
    ${catkin_EXPORTED_TARGETS}
  )
endif()
