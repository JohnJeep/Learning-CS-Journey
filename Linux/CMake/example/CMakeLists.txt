cmake_minimum_required(VERSION 3.20)

project(example)

add_subdirectory(src)

add_executable(example main.cpp )

target_link_libraries(example integer)