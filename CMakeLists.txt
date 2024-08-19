cmake_minimum_required(VERSION 3.26)
project(Kalman_Filter)

set(CMAKE_CXX_STANDARD 14)

include_directories(.
        eigen-3.3.8)

add_executable(Kalman_Filter
        Kalman.cpp
        Kalman.h
        main.cpp)
