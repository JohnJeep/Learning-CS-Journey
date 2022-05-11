<!--

 * @Author: JohnJeep
 * @Date: 2022-05-11 21:46:10
 * @LastEditTime: 2022-05-12 01:14:01
-->


# 1. CMake 教程

# 2. 语法特性

- 基本语法格式：命令(参数1 参数2 ...)
  - 参数使用花括号括起来。
  - 参数之间使用**空格**或**分号**分隔开。 

- 命令是不区分大小的，参数和变量是区分大小写的。

```cmake
# 给 test.cpp 设置一个变量 TEST
set(TEST test.cpp)

# 生成可执行文件需要依赖的内容
add_executable(test main.cpp test.cpp)
ADD_EXECUTABLe(test main.cpp test.cpp)  # 与上一条的用法一样
```

## 2.1. 变量

获取时使用 `${}` 的方式去取值，若在 `if` 语句中则需要直接使用变量名，不能使用 `${}` 的方式。
```cmake
set(TEST test.cpp)
```

## 2.2. CMake 命令

CMake 命令官方总共分为 4 大类。

- Scripting Commands
- Project Commands
- CTest Commands
- Deprecated Commands

下面仅仅列出常见的一些命令。

#### 2.2.1. cmake_minimum_required

```cmake
# 设置最低CMake版本要求
cmake_minimum_required(VERSION 3.12)
```

#### 2.2.2. project

设置工程的名字

```cmake
语法：
project(<PROJECT-NAME> [<language-name>...])
project(<PROJECT-NAME>
        [VERSION <major>[.<minor>[.<patch>[.<tweak>]]]]
        [DESCRIPTION <project-description-string>]
        [HOMEPAGE_URL <url-string>]
        [LANGUAGES <language-name>...])
        
# 项目名称
project(TestProj)
```

 设置工程的名字，并存储在变量 `TestProj` 中，当被上一级的 `CMakeLists.txt` 调用时，工程的名字被存在变量 `CMAKE_PROJECT_NAME` 中。

#### 2.2.3. set

编译器相关设置

```cmake
# 添加c++11标准支持
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")    

# 默认c编译器
SET(CMAKE_C_COMPILER "gcc.exe")

# 默认c++编译器
SET(CMAKE_CXX_COMPILER "g++.exe")

# 设置编译类型为 debug
set(CMAKE_BUILD_TYPE Debug)

# 设置编译类型为 release
set(CMAKE_BUILD_TYPE Release)
```

GDB 调试设置

```cmake
# Debug模式 选项: Release Debug MinSizeRel RelWithDebInfo
SET(CMAKE_BUILD_TYPE "Debug")

# debug模式下 gdb相关选项
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")  

# release模式下 gdb相关选项
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")  

# 开启调试 出现问题时开启
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
```

可执行文件

```cmake
# 设置可执行文件输出的目录
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)   
```

#### 2.2.4. add_definitions

```cmake
# 设置字符集
add_definitions(-DUNICODE -D_UTF-8) 
```

#### 2.2.5. add_compile_options

添加编译参数

```cmake
语法：add_compile_options(<option> ...)

# 添加编译参数 -Wall std=c++11 -O3
add_compile_options(-Wall std=c++11 -O3)
```



#### 2.2.6. add_library

生成库文件

```cmake
语法：add_library(libname [SHARED | STATIC | MODULE] [EXCULD_FROM_ALL] src1 src2 ...)

# 通过变量 SRC 生成 libtest.so 共享库，生成的时候会加上 lib 前缀和 .so 后缀
add_library(test SHARED ${SRC})
```

#### 2.2.7. add_executable

生成可执行文件

```cmake
# 指定 Src 目录下源文件生成的可执行文件的名字: main
add_executable(main ${SOURCES})
```

#### 2.2.8. add_subdirectory

向工程中添加存放源文件的子目录，作为可选项，可指定二进制文件或二进制文件存放的位置。

```cmake
语法：add_subdirectory(source_dir [binary_dir] [EXCLUDE_FROM_ALL])

# 工程中添加 google 子目录
# add_subdirectory(google)
```



#### 2.2.9. include_directories

```cmake
# 向工程中添加头文件路径，参数项可为一个或多个
语法：include_directories(dir1 dir2 ...)

include_directories(${TEST_PATH}/include)
```



#### 2.2.10. file

枚举头文件

```cmake
# 枚举头文件
file(GLOB_RECURSE INCLUDES "Inc/*.h" "Inc/*.hpp")  
```

#### 2.2.11. aux_source_directory

搜索指定目录 `dir` 下所有的源文件，并将结果列表存储在变量 `variable` 中。

```cmake
语法：aux_source_directory(<dir> <variable>)

# 搜索 Src 目录下所有的源文件，并将结果列表存储在变量 SOURCES 中
aux_source_directory(Src SOURCES) 
```

#### 2.2.12. link_directories

```cmake
# 依赖的链接库路径
link_directories(${TEST_PATH}/cmake/build)
```

#### 2.2.13. target_link_libraries

为目标文件链接所需要的共享库。

```cmake
语法：target_link_libraries(<target> ... <item>... ...)

# 添加链接库
# target_link_libraries(demo math) 
```

- `target` 目标名字必须通过 `add_executable()` 或 `add_library()` 命令创建的，不能是一个别名。
- `item` 可以下面几种类型。
  - **A library target name**：
  - **A full path to a library file**：
  - **A plain library name**：
  - **A link flag**: 
  - **A generator expression**。

#### 2.2.14. if/elseif

```cmake
# 区分操作系统
MESSAGE("Identifying the OS...")
if(WIN32)
  MESSAGE("This is Windows.")
elseif(APPLE)
  MESSAGE("This is MacOS.")
elseif(UNIX)
  MESSAGE("This is Linux.")
endif()
```

# 3. 构建方式

- 内部构建，不推荐使用

  内部构建会在同级目录产生一大堆中间文件，并放到和源工程同级的位置，但这些中间文件并不是我们所需要的，放在一起使工程显得杂乱无章，结构不清晰。

  ```cmake
  # 当前目录下编译本目录的 CMakeLists.txt 文件，生成 Makefile 和其它文件
  cmake .
  
  # 当前路径执行 make 命令，生成 target
  make
  ```

- 外部构建：推荐使用

  将编译输出的文件与源文件放到不同的目录中。

  ```cmake
  # 当前目录创建 build 文件夹
  mkdir build
  
  # 进入 build 目录
  cd build
  
  # 编译上级目录的 CMakeLists.txt 文件，生成 Makefile 和其它文件
  # cmake path，path 是上一级 CMakeLists.txt 文件的路径 
  cmake ..
  
  # 执行 make 命令，生成 target
  make
  ```

  

# 4. Reference

- [CMake Tutorial](https://cmake.org/cmake/help/latest/guide/tutorial/index.html)
- [CMake Commands](https://cmake.org/cmake/help/latest/manual/cmake-commands.7.html)

