cmake_minimum_required(VERSION 2.8.3)
project(mp_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  aggies_lib
  aggies_msgs
)

catkin_package(
  INCLUDE_DIRS include
  #  LIBRARIES mp_controller
  #  CATKIN_DEPENDS roscpp rospy std_msgs
  #  DEPENDS system_lib
)
include_directories(
  # /usr/local/include  
  ${OpenCV_INCLUDE_DIRS}  
)

# link_directories(/usr/local/lib)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  include/mp_controller
  ${catkin_INCLUDE_DIRS}
 
)

add_executable(
  mp_controller 
  src/mp_controller.cpp
  src/nlmpc.cpp
  src/fg_eval.cpp
)
# add_executable(
#   pure_pursuit
#   src/pure_pursuit.cpp
# )

add_dependencies(mp_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mp_controller ipopt ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
# target_link_libraries(pure_pursuit ${catkin_LIBRARIES})