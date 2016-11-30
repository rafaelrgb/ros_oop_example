cmake_minimum_required(VERSION 2.8.3)
project(ros_oop_example)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  mavros
  std_msgs
  mav_msgs
  nav_msgs
  geometry_msgs
 )

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp mavros std_msgs mav_msgs nav_msgs geometry_msgs
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

set(H_FILES
    include/Node.h
    include/RosOopExampleNode.h
)

set(CPP_FILES
    src/Node.cpp
    src/RosOopExampleNode.cpp
)

add_executable(ros_oop_example_node src/ros_oop_example_node.cpp ${CPP_FILES})
add_dependencies(ros_oop_example_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(ros_oop_example_node ${catkin_LIBRARIES})
