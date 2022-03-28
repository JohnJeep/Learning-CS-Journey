# 生成 Makefile 的规则

# cmake 支持的最小版本
cmake_minimum_required(VERSION 3.10)

# 工程名称
project(test)

# 设置变量：目录的前缀
set(TEST_PATH /home/John/test)

# 头文件路径
include_directories(${TEST_PATH}/include)

# 依赖的链接库路径
link_directories(${TEST_PATH}/cmake/build)

# 生成可执行文件需要依赖的内容
add_executable(test main.cpp test.cpp)

# 生成链接库
target_link_libraries(test ) 



