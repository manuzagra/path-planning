project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

include_directories(include Eigen-3.3)

add_library(trajectory SHARED
              src/trajectory.cpp
              src/cubic_trajectory.cpp)

set(sources
      src/main.cpp)


add_executable(path_planning ${sources})

target_link_libraries(path_planning trajectory z ssl uv uWS)
