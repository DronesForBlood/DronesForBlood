cmake_minimum_required(VERSION 3.5)

project(PATHPLANNER)
set (CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

<<<<<<< HEAD
find_package(OpenCV REQUIRED) 
find_package( catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation )

catkin_python_setup()

=======
find_package(OpenCV REQUIRED)
>>>>>>> develop
include_directories( SYSTEM
    ${OpenCV_INCLUDE_DIRS}
)


<<<<<<< HEAD
add_message_files(
  FILES
  start_end_coord.msg
  flight_mission.msg
  request.msg
  no_flight_circle.msg
  no_flight_area.msg
  utm_tracking_data.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# ADD ALL SOURCES AND HEADERS HERE
=======
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# ADD ALL SOURCES AND FREE HEADERS HERE
>>>>>>> develop
set(SOURCES
    sources/main.cpp
    sources/mapcontroller.cpp
    sources/mapgenerator.cpp
    sources/node.cpp
    sources/nodecollection.cpp
<<<<<<< HEAD
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


=======
    sources/pathfinder.cpp
    sources/simulation.cpp

    headers/defines.h
>>>>>>> develop
    headers/mapcontroller.h
    headers/mapgenerator.h
    headers/node.h
    headers/nodecollection.h
<<<<<<< HEAD
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



=======
    headers/pathfinder.h
    headers/simulation.h
)

>>>>>>> develop
add_executable(${PROJECT_NAME}
    ${SOURCES}
    )

<<<<<<< HEAD

add_dependencies(PATHPLANNER PATHPLANNER_generate_messages_cpp)

target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBRARIES}
    )

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    )



add_executable(SIM sources/sim.cpp)
target_link_libraries(SIM ${catkin_LIBRARIES})
add_dependencies(SIM PATHPLANNER_generate_messages_cpp)


#add_executable(UTM sources/UTM/get_uav_technical_data.py)
#target_link_libraries(UTM ${catkin_LIBRARIES})
#add_dependencies(UTM PATHPLANNER_generate_messages_cpp)




=======
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBRARIES}
    )
>>>>>>> develop
