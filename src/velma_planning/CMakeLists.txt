cmake_minimum_required(VERSION 2.8.3)
project(velma_planning)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  omnivelma_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  costmap_2d
  global_planner
  base_local_planner
  rotate_recovery
  dwa_local_planner
  teb_local_planner
  eband_local_planner
  tf2_geometry_msgs
  mbf_costmap_core
  turtlesim
)


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   make_plan.srv
#   Service1.srv
#   Service2.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   omnivelma_msgs
   nav_msgs
   turtlesim
)

catkin_package(
  INCLUDE_DIRS include /home/silver/INZYNIER/velma/ws_thesis/devel/include/velma_planning
  LIBRARIES velma_planning
  CATKIN_DEPENDS message_generation omnivelma_msgs roscpp rospy
  #DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(main_planner src/main.cpp)
target_link_libraries(main_planner ${catkin_LIBRARIES})