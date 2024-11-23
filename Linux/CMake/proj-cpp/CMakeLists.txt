# Requires CMake version 3.10 or higher
cmake_minimum_required(VERSION 3.10)

# Set the project name
project(proj-cpp)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 14)

# Require the C++ standard to be required
set(CMAKE_CXX_STANDARD_REQUIRED ture)

# Set the runtime output directory for executables
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Set the output directory for shared libraries
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Set the output directory for static libraries
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Add the include directory to the include path
include_directories(${CMAKE_SOURCE_DIR}/include)

# Add submodules
add_subdirectory(src)
add_subdirectory(tests)