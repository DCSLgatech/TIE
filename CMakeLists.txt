cmake_minimum_required(VERSION 2.8.3)
project(tie)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tie
CATKIN_DEPENDS roscpp rospy std_msgs roslib
#  DEPENDS system_lib
DEPENDS
    EIGEN3
)

find_package(Eigen3 REQUIRED)
find_package(OMPL REQUIRED)

if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

include_directories(SYSTEM ${Boost_INCLUDE_DIR}  ${OMPL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(src)
link_directories(${catkin_LIBRARY_DIRS})

# OMPLtools Library 
add_library(${PROJECT_NAME}_ompltools
  src/utilities/OMPLtools.cpp 
)
target_link_libraries(${PROJECT_NAME}_ompltools
  ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OMPL_LIBRARIES}
)

#TSST library
add_library(${PROJECT_NAME}_tsst
  src/tsst/TSST.cpp 
)
target_link_libraries(${PROJECT_NAME}_tsst
  ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OMPL_LIBRARIES}
)

###########
## Build ##
###########

SET( CMAKE_CXX_FLAGS  "-std=c++0x" )
SET( CMAKE_BUILD_TYPE Release)


add_executable(toy2d_propogate_test src/toy2d_propogate_test.cpp)
target_link_libraries(toy2d_propogate_test 
  ${OMPL_LIBRARIES} 
  ${catkin_LIBRARIES} 
  ${PROJECT_NAME}_ompltools
  ${PROJECT_NAME}_tsst
)



