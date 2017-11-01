cmake_minimum_required(VERSION 3.6)
project(Compiler)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

set(SOURCE_FILES main.cpp graph.h graph.cpp edge.h edge.cpp)
add_executable(main ${SOURCE_FILES} main.cpp graph.h graph.cpp)