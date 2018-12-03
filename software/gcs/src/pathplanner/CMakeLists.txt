cmake_minimum_required(VERSION 3.5)

project(pathplanner)
set (CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

find_package(OpenCV REQUIRED) 
find_package( catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation mavlink_lora utm)

include_directories( SYSTEM
    ${OpenCV_INCLUDE_DIRS}
)


add_message_files(
  FILES
  start_end_coord.msg
  flight_mission.msg
  blocked_goal.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
  mavlink_lora
  utm
)

set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# ADD ALL SOURCES AND HEADERS HERE
set(SOURCES
    sources/main.cpp
    sources/mapcontroller.cpp
    sources/mapgenerator.cpp
    sources/node.cpp
    sources/nodecollection.cpp
    sources/pathexporter.cpp
    sources/pathfinder.cpp
    sources/pathshortener.cpp
    sources/simulation.cpp

    sources/rosMsg.cpp
    sources/visualizer.cpp

    sources/watch/watchdrone.cpp
    sources/watch/watchzone.cpp


    headers/mapcontroller.h
    headers/mapgenerator.h
    headers/node.h
    headers/nodecollection.h
    headers/pathexporter.h
    headers/pathfinder.h
    headers/pathshortener.h
    headers/simulation.h

    headers/rosMsg.h
    headers/visualizer.h

    headers/watch/watchdrone.h
    headers/watch/watchzone.h

    headers/global/clipper.cpp
    headers/global/clipper.hpp
    headers/global/coordconverter.h
    headers/global/defines.h
    headers/global/geofunctions.h

)


include_directories(
${catkin_INCLUDE_DIRS} 
)



add_executable(${PROJECT_NAME}
    ${SOURCES}
    )


add_dependencies(pathplanner pathplanner_generate_messages_cpp)

add_dependencies(pathplanner ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBRARIES}
    )

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    )


add_executable(SIM sources/sim.cpp)
target_link_libraries(SIM ${catkin_LIBRARIES})
add_dependencies(SIM pathplanner_generate_messages_cpp)




