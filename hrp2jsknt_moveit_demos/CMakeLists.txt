cmake_minimum_required(VERSION 2.8.3)
project(hrp2jsknt_moveit_demos)

find_package(catkin REQUIRED COMPONENTS 
  moveit_core 
  moveit_ros_planning 
  moveit_ros_planning_interface 
  pluginlib
  block_grasp_generator
  hrp2_moveit_msgs
)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    moveit_core
    moveit_ros_planning_interface
    moveit_msgs
    interactive_markers
    block_grasp_generator
)

###########
## Build ##
###########
include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(hrp2_demos src/hrp2_demos.cpp)
target_link_libraries(hrp2_demos ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(hrp2_demos hrp2_moveit_msgs_gencpp ${PROJECT_NAME}_gencfg) # wait for external msgs to be built