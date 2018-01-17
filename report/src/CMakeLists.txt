cmake_minimum_required(VERSION 3.5)
project(Compiler)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -std=c++14 -O2")

set(SOURCE_FILES main.cpp graph.h edge.h edge.cpp)
add_executable(main ${SOURCE_FILES} main.cpp graph.h graph.cpp)
