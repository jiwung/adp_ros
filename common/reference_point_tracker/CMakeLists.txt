cmake_minimum_required(VERSION 3.0)
project(reference_point_tracker)

##################
## Dependencies ##
##################
find_package(catkin REQUIRED COMPONENTS
    trajectory
    visualization_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
  trajectory
    visualization_msgs
  DEPENDS
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
)

find_package(Boost REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(GTest QUIET)

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

set(PROJECT_SRC
    src/${PROJECT_NAME}.cpp
#    src/${PROJECT_NAME}_vis.cpp
    )

add_library(${PROJECT_NAME}
    ${PROJECT_SRC}
    )
target_link_libraries(${PROJECT_NAME}
    Eigen3::Eigen
    ${catkin_LIBRARIES}
)

##########
## Test ##
##########

enable_testing()
add_executable(${PROJECT_NAME}_test
  test/${PROJECT_NAME}_test.cpp
  )
target_link_libraries(${PROJECT_NAME}_test
  ${PROJECT_NAME}
  ${GTEST_LIBRARIES}
  ${catkin_LIBRARIES}
  pthread
  )

#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
