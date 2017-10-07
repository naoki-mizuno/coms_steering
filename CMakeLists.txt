cmake_minimum_required(VERSION 2.8.3)
project(coms_steering)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  serial
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  LIBRARIES coms_steering SteeringController
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs serial
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)

add_library(
  SteeringController
  src/SteeringController.cpp
  include/coms_steering/SteeringController.h
)

add_library(
  coms_steering
  src/coms_steering.cpp
  include/coms_steering/coms_steering.h
)

add_executable(coms_steering_node src/coms_steering_node.cpp)

target_link_libraries(coms_steering_node
  coms_steering
  SteeringController
  ${catkin_LIBRARIES}
)

install(TARGETS coms_steering SteeringController coms_steering_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/launch
)
