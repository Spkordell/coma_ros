cmake_minimum_required(VERSION 2.8.3)
project(coma_kinematics)


## use C++ 11 ##needed for std::bind
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp cmake_modules std_msgs message_generation)
find_package(Ceres REQUIRED)
find_package(Eigen REQUIRED)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  solveIK.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

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
  ${CERES_INCLUDES}
)

## Declare a cpp library
add_library(coma_kinematics src/cosserat_rod.cpp)
target_link_libraries(coma_kinematics ${catkin_LIBRARIES})

## Declare a cpp executable
add_executable(ik src/ik.cpp)
target_link_libraries(ik coma_kinematics ${catkin_LIBRARIES} ${CERES_LIBRARIES})




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
