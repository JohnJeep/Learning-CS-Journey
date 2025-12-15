cmake_minimum_required(VERSION 3.5)
project(genertic_type)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(behaviortree_cpp REQUIRED)

add_executable(${PROJECT_NAME}
  genertic_type.cpp
)

target_link_libraries(${PROJECT_NAME} PRIVATE
  behaviortree_cpp::behaviortree_cpp
)