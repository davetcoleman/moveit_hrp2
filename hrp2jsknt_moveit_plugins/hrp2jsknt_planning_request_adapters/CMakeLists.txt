set(MOVEIT_LIB_NAME hrp2jsknt_planning_request_adapters)

set(SOURCE_FILES
  src/add_walking_steps.cpp)

add_library(${MOVEIT_LIB_NAME} ${SOURCE_FILES})
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

