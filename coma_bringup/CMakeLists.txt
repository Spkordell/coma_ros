cmake_minimum_required(VERSION 2.8.3)
project(coma_bringup)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}	
)

## Declare a cpp executable
##add_executable(motion_demo src/motion_demo.cpp)
##target_link_libraries(motion_demo ${catkin_LIBRARIES})


#############
## Install ##
#############

## Mark executables and/or libraries for installation
##install(TARGETS coma_joy_teleop
##  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
##)

## Copy header files
##install(DIRECTORY include/${PROJECT_NAME}/
##  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
##  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
##)

## Copy launch files
##install(DIRECTORY launch/
##  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
##)
