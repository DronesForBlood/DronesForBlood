cmake_minimum_required(VERSION 3.5)

project(PATHPLANNER)
set (CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

find_package(OpenCV REQUIRED)
include_directories( SYSTEM
    ${OpenCV_INCLUDE_DIRS}
)


set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# ADD ALL SOURCES AND FREE HEADERS HERE
set(SOURCES
    sources/main.cpp
    sources/mapcontroller.cpp
    sources/mapgenerator.cpp
    sources/node.cpp
    sources/nodecollection.cpp
    sources/pathfinder.cpp
    sources/simulation.cpp

    headers/defines.h
    headers/mapcontroller.h
    headers/mapgenerator.h
    headers/node.h
    headers/nodecollection.h
    headers/pathfinder.h
    headers/simulation.h
)

add_executable(${PROJECT_NAME}
    ${SOURCES}
    )

target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBRARIES}
    )
