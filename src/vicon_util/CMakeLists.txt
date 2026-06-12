cmake_minimum_required(VERSION 3.8)
project(vicon_util)

add_compile_options(-std=c++17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(DEPENDENCIES
  roscpp
  derived_object_msgs
  nav_msgs
  geometry_msgs
  ros_tools
)

# find dependencies
find_package(catkin REQUIRED COMPONENTS
  ${DEPENDENCIES}
)

catkin_package(
  CATKIN_DEPENDS ${DEPENDENCIES}
)

catkin_install_python(PROGRAMS scripts/launch_ekfs.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(obstacle_bundle_node src/obstacle_bundle_node.cpp)
add_dependencies(obstacle_bundle_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(obstacle_bundle_node ${catkin_LIBRARIES})

add_executable(spoof_vicon_node src/spoof_vicon_node.cpp)
add_dependencies(spoof_vicon_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(spoof_vicon_node ${catkin_LIBRARIES})

add_executable(visualize_scene src/visualize_scene.cpp)
add_dependencies(visualize_scene ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(visualize_scene ${catkin_LIBRARIES})
add_definitions(-DMPC_PLANNER_ROS)

install(TARGETS
  obstacle_bundle_node
  spoof_vicon_node
  visualize_scene
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)