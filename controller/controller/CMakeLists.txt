cmake_minimum_required(VERSION 2.8.3)
project(controller)

##################
## Dependencies ##
##################
find_package(catkin REQUIRED COMPONENTS
    roscpp
    eigen_conversions
    ackermann_msgs
    nav_msgs
    trajectory
    reference_point_tracker
    pure_pursuit
    rosbag)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    roscpp
    eigen_conversions
    ackermann_msgs
    nav_msgs
    trajectory
    reference_point_tracker
    pure_pursuit
  DEPENDS
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME})

find_package(Boost REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(GTest QUIET)

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS})

set(PROJECT_SRC
    src/${PROJECT_NAME}.cpp
    )

add_library(${PROJECT_NAME}_core
    ${PROJECT_SRC})
target_link_libraries(${PROJECT_NAME}_core
    Eigen3::Eigen
    ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}
    src/${PROJECT_NAME}_ros.cpp
    src/main.cpp)
target_link_libraries(${PROJECT_NAME}
    ${PROJECT_NAME}_core)

##########
## Test ##
##########

enable_testing()

add_executable(${PROJECT_NAME}_test
  test/${PROJECT_NAME}_test.cpp)
target_link_libraries(${PROJECT_NAME}_test
  ${PROJECT_NAME}_core
  ${GTEST_LIBRARIES}
  ${catkin_LIBRARIES}
  pthread)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
