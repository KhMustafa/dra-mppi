cmake_minimum_required(VERSION 2.8.3)
project(asr_rapidxml)

include_directories(include)

find_package(catkin)

catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES ${PROJECT_NAME}
)

add_library(${PROJECT_NAME} STATIC     
    include/rapidxml.hpp
    include/rapidxml_iterators.hpp
    include/rapidxml_print.hpp
    include/rapidxml_print.hpp)
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/
	DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

