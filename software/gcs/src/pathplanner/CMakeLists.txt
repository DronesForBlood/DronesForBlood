cmake_minimum_required(VERSION 3.5)

project(PATHPLANNER)
set (CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

find_package(OpenCV REQUIRED) 
find_package( catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation mavlink_lora)

catkin_python_setup()

include_directories( SYSTEM
    ${OpenCV_INCLUDE_DIRS}
)


add_message_files(
  FILES
  start_end_coord.msg
  flight_mission.msg
  request.msg
  no_flight_circle.msg
  no_flight_area.msg
  utm_tracking_data.msg

  #mavlink_lora_pos.msg
  #mavlink_lora_mission_list.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  mavlink_lora
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

    sources/UTM/get_uav_technical_data.py
    sources/UTM/add_uav_tracking_data.py
    sources/UTM/utm_node.py
    sources/UTM/kml_reader.py


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

    headers/global/coordconverter.h
    headers/global/defines.h
    headers/global/geofunctions.h

)


include_directories(
${catkin_INCLUDE_DIRS} 
) 

install(PROGRAMS
     sources/UTM/get_uav_technical_data.py
     sources/UTM/add_uav_tracking_data.py
     sources/UTM/utm_node.py
     sources/UTM/kml_reader.py
     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



add_executable(${PROJECT_NAME}
    ${SOURCES}
    )


add_dependencies(PATHPLANNER PATHPLANNER_generate_messages_cpp)

add_dependencies(PATHPLANNER ${catkin_EXPORTED_TARGETS})
#add_dependencies(mavlink_lora_node mavlink_lora_generate_messages_cpp)

target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBRARIES}
    )

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    )



add_executable(SIM sources/sim.cpp)
target_link_libraries(SIM ${catkin_LIBRARIES})
add_dependencies(SIM PATHPLANNER_generate_messages_cpp)




