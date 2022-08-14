cmake_minimum_required(VERSION 3.5)
project(param_util)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    dynamic_reconfigure
    roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS})

add_executable(example_node example/example_node.cpp)
target_link_libraries(example_node ${catkin_LIBRARIES})
set_property(TARGET example_node PROPERTY CXX_STANDARD 17)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
