cmake_minimum_required(VERSION 2.8.3)
project(pedsim_original)

#------------------ Configuration ------------------#
option(SHALL_DEBUG "Enable debug features" OFF)
option(SHALL_PROFILE "Enable the code profiling feature" OFF)
option(CMAKE_VERBOSE_MAKEFILE "Full compiler output" ON)


#----------------- Compiler Flags ------------------#
add_definitions(-Wall -Wunused -std=c++0x -pipe)

#debugging
if(SHALL_DEBUG)
  message("Debugging activated")
  add_definitions(-O0 -DDEBUG -ggdb -g3 -rdynamic)
else(SHALL_DEBUG)
  message("Debugging deactivated")
  add_definitions(-Os)
endif(SHALL_DEBUG)


find_package(catkin REQUIRED COMPONENTS
  roscpp
)
find_package(Boost REQUIRED)

catkin_package(
  CATKIN_DEPENDS  roscpp
  INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include/
  LIBRARIES pedsim_original
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${roscpp_INCLUDE_DIRS}
)

set(SOURCES
  src/ped_agent.cpp
  src/ped_angle.cpp
  src/ped_obstacle.cpp
  src/ped_scene.cpp
  src/ped_tree.cpp
  src/ped_vector.cpp
  src/ped_waypoint.cpp
)

add_library(pedsim_original ${SOURCES})

target_link_libraries(pedsim_original
  ${BOOST_LIBRARIES}
)

install(TARGETS pedsim_original
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)