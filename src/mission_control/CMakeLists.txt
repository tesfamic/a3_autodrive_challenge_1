cmake_minimum_required(VERSION 2.8.3)
project(mission_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  aggies_msgs
  aggies_lib
)

## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES mission_control
  CATKIN_DEPENDS roscpp rospy std_msgs #message_runtime
  #  DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  include/mission_control
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  mission_control 
  src/mission_control.cpp
  src/mission.cpp
  src/route_planner.cpp
  src/traj_plan.cpp #trajectory_planner.cpp
  src/a3_timer.cpp
  src/wpt_follower.cpp

)
target_link_libraries(
  mission_control
   ${catkin_LIBRARIES}
)

add_dependencies(
  mission_control 
  ${catkin_LIBRARIES}
)


