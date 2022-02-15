cmake_minimum_required(VERSION 3.0.2)
project(traffic_light)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(OpenCV 4.2 REQUIRED)
find_package(InferenceEngine REQUIRED)
if(NOT(InferenceEngine_FOUND))
   message(FATAL_ERROR ${IE_NOT_FOUND_MESSAGE})
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  aggies_msgs
)

catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES traffic_light
  CATKIN_DEPENDS cv_bridge image_transport roscpp rospy
  #  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  include/traffic_light
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${InferenceEngine_INCLUDE_DIRS}
  
)
add_executable(
  traffic_light 
  src/traffic_light.cpp
)

target_link_libraries(
  traffic_light
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} 
  ${InferenceEngine_LIBRARIES}
)


# add_dependencies(tl_classifier ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} pthread)




