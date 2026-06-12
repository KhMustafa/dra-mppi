cmake_minimum_required(VERSION 2.8.3)
project(roadmap_msgs)

find_package(catkin REQUIRED COMPONENTS std_msgs geometry_msgs message_generation)

include_directories(include)

add_message_files(
  DIRECTORY msg
  FILES RoadPolyline.msg RoadPolylineArray.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  DEPENDS std_msgs geometry_msgs
)

install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
