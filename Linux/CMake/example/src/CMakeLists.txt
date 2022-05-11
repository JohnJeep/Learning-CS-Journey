include_directories(${CMAKE_SOURCE_DIR}/include)

add_library(integer SHARED integer.cpp)

add_executable(example main.cpp )

target_link_libraries(example integer)