cmake_minimum_required(VERSION 3.5)

project(PATH_FINDER)
set (CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")


set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# ADD ALL SOURCES AND FREE HEADERS HERE
set(SOURCES
    sources/main.cpp
    sources/node.cpp
    sources/pathfinder.cpp

    headers/node.h
    headers/pathfinder.h
)

add_executable(${PROJECT_NAME}
    ${SOURCES}
    )

target_link_libraries(${PROJECT_NAME}
    )
