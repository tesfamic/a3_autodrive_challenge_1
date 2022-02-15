cmake_minimum_required(VERSION 2.8.3)
project(lane_detector)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(PkgConfig REQUIRED)

find_package(OpenCV 4. REQUIRED)
find_package(InferenceEngine REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  aggies_msgs
  aggies_lib
  # message_generation
)
find_package(PkgConfig REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  #  LIBRARIES lane_detector
  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs #message_runtime
  #  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  include/lane_detector
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  
)

add_executable(
  cnn_lane_detector
  src/cnn_lane_detector.cpp
  src/detector_cnn.cpp
)
add_executable(
  lane_detector
  src/lane_detector.cpp
  src/detector.cpp
)

target_link_libraries(lane_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(cnn_lane_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

