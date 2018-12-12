cmake_minimum_required(VERSION 3.5)

project(utm)

find_package( catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation drone_decon)

catkin_python_setup()

add_message_files(
  FILES
  utm_tracking_data.msg
  utm_no_flight_area.msg
  utm_no_flight_circle.msg
  utm_rally_point.msg
  utm_rally_point_list.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  drone_decon
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
    src/main.cpp
    src/utm_node.py
    src/kml_reader.py
)


add_executable(utm
    ${SOURCES}
    )

include_directories(
${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
     src/utm_node.py
     src/kml_reader.py
     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_dependencies(utm utm_generate_messages_cpp)

add_dependencies(utm ${catkin_EXPORTED_TARGETS})


target_link_libraries(utm
    ${catkin_LIBRARIES}
    )



