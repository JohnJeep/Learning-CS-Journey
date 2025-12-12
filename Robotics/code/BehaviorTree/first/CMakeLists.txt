cmake_minimum_required(VERSION 3.5)
project(first_bt)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(behaviortree_cpp REQUIRED)
message(STATUS "Version: ${behaviortree_cpp_VERSION}")

if(behaviortree_cpp_FOUND)
    message(STATUS "BehaviorTree.CPP found at: ${behaviortree_cpp_DIR}")
    message(STATUS "BehaviorTree.CPP include dirs: ${behaviortree_cpp_INCLUDE_DIRS}")
    message(STATUS "BehaviorTree.CPP libraries: ${behaviortree_cpp_LIBRARIES}")
    message(STATUS "BehaviorTree.CPP traget: ${behaviortree_cpp_TARGETS}")

#     get_cmake_property(_variableNames VARIABLES)
# foreach (_variableName ${_variableNames})
#     if (_variableName MATCHES "behaviortree_cpp")
#         message(STATUS "${_variableName}=${${_variableName}}")
#     endif()
# endforeach()

endif()

add_executable(${PROJECT_NAME}
  first_bt.cpp
)

target_link_libraries(${PROJECT_NAME} PRIVATE
  behaviortree_cpp::behaviortree_cpp
)