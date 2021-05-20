# 最低CMake版本要求
cmake_minimum_required(VERSION 3.12)

# 项目名称
project(CMakeTest)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")             # 添加c++11标准支持

SET(CMAKE_C_COMPILER "gcc.exe")                                  # 默认c编译器
SET(CMAKE_CXX_COMPILER "g++.exe")                                # 默认c++编译器
add_definitions(-DUNICODE -D_UTF-8)                              # 设置字符集

# 使用GDB调试
SET(CMAKE_BUILD_TYPE "Debug")                                    # Debug模式 选项: Release Debug MinSizeRel RelWithDebInfo
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")  # debug模式下 gdb相关选项
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")          # release模式下 gdb相关选项
# set(CMAKE_CXX_STANDARD_REQUIRED ON)                            # 开启调试 出现问题时开启


INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/Inc")                 # 头文件包含目录
# file(GLOB_RECURSE INCLUDES "Inc/*.h" "Inc/*.hpp")              # 枚举头文件
aux_source_directory(Src SOURCES)                                # 搜索 Src 目录下所有的源文件，并将结果列表存储在变量 SOURCES 中
add_executable(main ${SOURCES})                                  # 指定 Src 目录下源文件生成的可执行文件的名字: main
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)            # 设置可执行文件输出的目录

# 多文件单子目录测试
# add_subdirectory(math)  # 添加math子目录
# target_link_libraries(demo math)  # 添加链接库


# 这段代码可以区分操作系统
MESSAGE("Identifying the OS...")
if(WIN32)
  MESSAGE("This is Windows.")
elseif(APPLE)
  MESSAGE("This is MacOS.")
elseif(UNIX)
  MESSAGE("This is Linux.")
endif()
