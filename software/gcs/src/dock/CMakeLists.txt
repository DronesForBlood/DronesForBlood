cmake_minimum_required(VERSION 3.5)

project(dock)

find_package( catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
)

set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# ADD ALL SOURCES AND HEADERS HERE
set(SOURCES
    src/dock_node.py
)

include_directories(
${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
     src/dock_node.py
     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
