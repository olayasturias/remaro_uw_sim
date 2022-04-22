cmake_minimum_required(VERSION 3.0.2)
project(gazebodrone)

# set this to path to AirSim root folder if you want your catkin workspace in a custom directory
set(AIRSIM_ROOT $ENV{HOME}/dev/AirSim)

add_subdirectory("${AIRSIM_ROOT}/cmake/rpclib_wrapper" rpclib_wrapper)
add_subdirectory("${AIRSIM_ROOT}/cmake/AirLib" AirLib)
add_subdirectory("${AIRSIM_ROOT}/cmake/MavLinkCom" MavLinkCom)


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(gazebo REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${AIRSIM_ROOT}/AirLib/deps/eigen3
  ${AIRSIM_ROOT}/AirLib/deps/rpclib/include
  ${AIRSIM_ROOT}/AirLib/include
  ${RPC_LIB_INCLUDES}
)

link_directories(
    ${GAZEBO_LIBRARY_DIRS}
    ../../build_release/output/lib
)

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gazebodrone
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(GazeboDrone src/GazeboDrone.cpp)
target_link_libraries(GazeboDrone ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} AirLib pthread rpc)

install(TARGETS
    #list of nodes
    GazeboDrone
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
