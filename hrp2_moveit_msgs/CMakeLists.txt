cmake_minimum_required(VERSION 2.8.3)
project(hrp2_moveit_msgs)

set(MSG_DEPS
  moveit_msgs
)

find_package(catkin REQUIRED genmsg ${MSG_DEPS})

set(SRV_FILES
GetWalkingMotion.srv
)

add_service_files(DIRECTORY srv FILES ${SRV_FILES})

generate_messages(DEPENDENCIES ${MSG_DEPS})

catkin_package(DEPENDS ${MSG_DEPS})
