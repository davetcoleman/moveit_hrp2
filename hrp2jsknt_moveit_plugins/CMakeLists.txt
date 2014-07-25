cmake_minimum_required(VERSION 2.8.3)
project(hrp2jsknt_moveit_plugins)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED thread)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  moveit_visual_tools
  moveit_msgs
  hrp2_moveit_msgs
  pluginlib
  roscpp
)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
    hrp2jsknt_moveit_constraint_sampler/include/
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning
    moveit_visual_tools
    hrp2_moveit_msgs
)

include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
include_directories(
  hrp2jsknt_moveit_constraint_sampler/include/
  ${catkin_INCLUDE_DIRS}
)

link_directories(${Boost_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_subdirectory(hrp2jsknt_moveit_constraint_sampler)
add_subdirectory(hrp2jsknt_planning_request_adapters)

install(
  FILES
    hrp2jsknt_moveit_constraint_sampler_plugin_description.xml
    hrp2jsknt_planning_request_adapters_plugin_description.xml
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

