cmake_minimum_required(VERSION 3.13)
project(potatoes)

set(CMAKE_CXX_STANDARD 14)

include_directories(include)
include_directories(src)
include_directories()

add_executable(potatoes
        include/API.h
        opcontrol.c
        robot.c)