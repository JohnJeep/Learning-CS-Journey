cmake_minimum_required(VERSION 3.5)
project(hello)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set(CMAKE_PREFIX_PATH "/usr/local/lib/cmake/behaviortree_cpp_v3" ${CMAKE_PREFIX_PATH})
find_package(behaviortree_cpp_v3 REQUIRED)
message(STATUS "   Version: ${behaviortree_cpp_v3_version}")

add_executable(${PROJECT_NAME}
  hello.cpp
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    BT::behaviortree_cpp_v3
)