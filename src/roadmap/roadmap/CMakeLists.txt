cmake_minimum_required(VERSION 3.0.2)
project(roadmap)

add_compile_options(-std=c++17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(DEPENDENCIES
  geometry_msgs
  roslib
  roscpp
  std_msgs
  asr_rapidxml
  roadmap_msgs
  ros_tools
)

find_package(catkin REQUIRED COMPONENTS
  ${DEPENDENCIES}
)

catkin_package(
  CATKIN_DEPENDS ${DEPENDENCIES}
  INCLUDE_DIRS include include/${PROJECT_NAME} include/spline
  LIBRARIES roadmap
)

include_directories(
  include
  include/roadmap
  include/spline
  ${catkin_INCLUDE_DIRS}
)

# # Declare a C++ library
add_library(roadmap
  src/roadmap.cpp
  src/configuration.cpp
  src/reader.cpp
  src/spline_converter.cpp
  src/spline/spline.cpp
  src/spline/Clothoid.cpp
)
add_dependencies(roadmap ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(roadmap_node src/roadmap_node.cpp)
add_dependencies(roadmap_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(roadmap_node roadmap ${catkin_LIBRARIES})

add_definitions(-DMPC_PLANNER_ROS)

install(TARGETS roadmap_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)