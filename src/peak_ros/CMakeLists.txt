cmake_minimum_required(VERSION 3.0.2)
project(peak_ros)

add_compile_options(-std=c++17)
#set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wconversion -Wunused -Wunreachable-code")

# Required for compiled dependencies
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(FetchContent)
FetchContent_Declare(
  peak_micropulse
  GIT_REPOSITORY https://github.com/MShields1986/peak_micropulse_driver
  GIT_TAG        feature/nodelet_compatibility
)
FetchContent_MakeAvailable(peak_micropulse)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf2_msgs
  tf2_geometry_msgs
  rosbag
  pcl_ros
  pcl_conversions
  nodelet
  message_generation
  dynamic_reconfigure
)

find_package(PCL REQUIRED)

find_package(Boost REQUIRED COMPONENTS system filesystem)

# Hardcode the full path to the shared fftw3 library to bypass the static
# libfftw3.a in /usr/local which was not compiled with -fPIC.
set(FFTW3_INCLUDE_DIRS /usr/include)
set(FFTW3_SHARED_LIB /usr/lib/x86_64-linux-gnu/libfftw3.so)
if(NOT EXISTS ${FFTW3_SHARED_LIB})
  message(FATAL_ERROR "Expected shared FFTW3 library not found at ${FFTW3_SHARED_LIB}")
endif()
message(STATUS "Using FFTW3: ${FFTW3_SHARED_LIB}")

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Ascan.msg
  Observation.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  StreamData.srv
  TakeSingleMeasurement.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

generate_dynamic_reconfigure_options(
  config/dynamic_variables.cfg
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    nodelet
    pluginlib
    message_generation
    message_runtime
    std_msgs
    sensor_msgs
    geometry_msgs
    visualization_msgs
    tf2
    tf2_msgs
    tf2_ros
    tf2_sensor_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${FFTW3_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(
  ${PCL_DEFINITIONS}
)

###################
# Nodelet Library #
###################

add_library(
  ${PROJECT_NAME}_nodelets
  src/peak_nodelet.cpp
  src/reconstruction_nodelet.cpp
)

add_dependencies(${PROJECT_NAME}_nodelets ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_nodelets
  ${catkin_LIBRARIES}
  /usr/lib/x86_64-linux-gnu/libfftw3.so
  PeakMicroPulseHandler
)

###############
# Peak Node   #
###############

set(EXECUTABLE_NAME peak_node)
set(EXECUTABLE_SRC src/peak_node.cpp)

add_executable(${EXECUTABLE_NAME} ${EXECUTABLE_SRC})
add_executable(ascan_postprocessor src/ascan_postprocessor.cpp)

add_dependencies(ascan_postprocessor
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(ascan_postprocessor
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_dependencies(${EXECUTABLE_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${EXECUTABLE_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  PeakMicroPulseHandler
)

#######################
# Reconstruction Node #
#######################

set(EXECUTABLE_NAME reconstruction_node)
set(EXECUTABLE_SRC src/reconstruction_node.cpp)

add_executable(${EXECUTABLE_NAME} ${EXECUTABLE_SRC})
add_dependencies(${EXECUTABLE_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${EXECUTABLE_NAME} ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS ascan_postprocessor
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)