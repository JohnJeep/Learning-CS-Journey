<!--

 * @Author: JohnJeep
 * @Date: 2022-05-11 21:46:10
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-01-01 17:55:37
 * @Description: CMake useage
 * Copyright (c) 2025 by John Jeep, All Rights Reserved. 
-->

- [1. Introduction](#1-introduction)
- [2. variables](#2-variables)
- [3. Commands](#3-commands)
  - [3.1. Scripting Commands](#31-scripting-commands)
    - [3.1.1. cmake\_minimum\_required](#311-cmake_minimum_required)
    - [3.1.2. messge](#312-messge)
    - [3.1.3. set](#313-set)
    - [3.1.4. unset](#314-unset)
    - [3.1.5. if/elseif](#315-ifelseif)
    - [3.1.6. file](#316-file)
    - [3.1.7. option](#317-option)
  - [3.2. Project Commands](#32-project-commands)
    - [3.2.1. project](#321-project)
    - [3.2.2. add\_definitions](#322-add_definitions)
    - [3.2.3. add\_compile\_options](#323-add_compile_options)
    - [3.2.4. add\_library](#324-add_library)
    - [3.2.5. add\_executable](#325-add_executable)
    - [3.2.6. add\_subdirectory](#326-add_subdirectory)
    - [3.2.7. include\_directories](#327-include_directories)
    - [3.2.8. link\_directories](#328-link_directories)
    - [3.2.9. target\_sources](#329-target_sources)
    - [3.2.10. target\_include\_directories](#3210-target_include_directories)
    - [3.2.11. target\_link\_libraries](#3211-target_link_libraries)
    - [3.2.12. install](#3212-install)
      - [3.2.12.1. 语法](#32121-语法)
      - [3.2.12.2. 安装前缀(prefix)](#32122-安装前缀prefix)
      - [3.2.12.3. 安装目标(TARGETS)](#32123-安装目标targets)
      - [3.2.12.4. 安装普通文件(FILES)](#32124-安装普通文件files)
      - [3.2.12.5. 安装目录(DIRECTORY)](#32125-安装目录directory)
      - [3.2.12.6. 安装程序(PROGRAMS)](#32126-安装程序programs)
      - [3.2.12.7. 安装脚本或代码(SCRIPT / CODE)](#32127-安装脚本或代码script--code)
      - [3.2.12.8. 导出目标供其他项目使用（EXPORT）](#32128-导出目标供其他项目使用export)
      - [3.2.12.9. 组件(COMPONENT)](#32129-组件component)
      - [3.2.12.10. 示例](#321210-示例)
- [4. Build](#4-build)
  - [4.1. 内部构建](#41-内部构建)
  - [4.2. 外部构建](#42-外部构建)
- [5. Usage](#5-usage)
- [6. References](#6-references)

# 1. Introduction

CMake 是一个开源的、跨平台的**构建系统生成器**（build system generator），用于管理软件项目的编译过程。它本身并不直接编译代码，而是根据项目配置文件（通常是 `CMakeLists.txt`）生成适用于特定平台和编译器的构建文件（如 Makefile、Visual Studio 项目、Xcode 项目、Ninja 构建脚本等）。

可以把CMake 理解成是一门脚本语言。

**主要特点：**

- **跨平台**：支持 Windows、Linux、macOS 等多种操作系统。
- **多编译器支持**：可与 GCC、Clang、MSVC 等主流编译器配合使用。
- **语言支持广泛**：主要面向 C/C++，但也支持 Fortran、CUDA、Objective-C、Swift 等。
- **模块化与可扩展**：提供丰富的内置命令和模块，也支持用户自定义函数和宏。
- **集成开发环境友好**：能生成 IDE 项目文件，便于在 Visual Studio、CLion、Qt Creator 等工具中使用。

**基本使用流程：**

1. 编写 `CMakeLists.txt` 文件，描述项目结构、源文件、依赖、编译选项等。

2. 在构建目录中运行：

   ```bash
   cmake /path/to/source
   ```

   这会生成对应平台的构建系统（如 Makefile）。

3. 使用生成的构建系统进行编译，例如：

   ```bash
   make
   ```

   或者如果使用 Ninja：

   ```bash
   ninja
   ```

示例 `CMakeLists.txt`：

```cmake
cmake_minimum_required(VERSION 3.10)
project(MyApp)

set(CMAKE_CXX_STANDARD 17)

add_executable(myapp main.cpp)
```

这个简单的配置定义了一个名为 `MyApp` 的项目，使用 C++17 标准，并从 `main.cpp` 构建一个可执行文件 `myapp`。

------

CMake 因其灵活性和强大功能，已成为现代 C/C++ 项目的事实标准构建工具。

# 2. variables

使用 `${}` 的方式获取变量名。

**特性**

1. CMake 中的变量区分大小写。
2. 分类

   - 内置变量： **CMake 内置变量默认是大写字母命名**，通常采用全大写加下划线的形式。

   - 自定义变量：为了与 CMake 内置变量区分开，**推荐用户自定义变量使用小写或驼峰命名**。也就是说：**缓存变量（cache variables）** 和 **环境变量** 建议大写（如内置变量大多属于此类）；**普通局部变量** 建议小写。
3. 变量在 CMake 中存储时都是字符串。有多个变量时，内部存储使用 `；` 进行分割，但显示的时候不会显示分号`；`。
4. 对变量的基础操作：使用  `set()` 和 `unset()` 命令。



# 3. Commands

CMake 命令官方总共分为 4 大类。CMake 的  **commands 不区分大小。**

官方文档：https://cmake.org/cmake/help/latest/manual/cmake-commands.7.html

- Scripting Commands
  - [block](https://cmake.org/cmake/help/latest/command/block.html)
  - [break](https://cmake.org/cmake/help/latest/command/break.html)
  - [cmake_host_system_information](https://cmake.org/cmake/help/latest/command/cmake_host_system_information.html)
  - [cmake_language](https://cmake.org/cmake/help/latest/command/cmake_language.html)
  - [cmake_minimum_required](https://cmake.org/cmake/help/latest/command/cmake_minimum_required.html)
  - [cmake_parse_arguments](https://cmake.org/cmake/help/latest/command/cmake_parse_arguments.html)
  - [cmake_path](https://cmake.org/cmake/help/latest/command/cmake_path.html)
  - [cmake_pkg_config](https://cmake.org/cmake/help/latest/command/cmake_pkg_config.html)
  - [cmake_policy](https://cmake.org/cmake/help/latest/command/cmake_policy.html)
  - [configure_file](https://cmake.org/cmake/help/latest/command/configure_file.html)
  - [continue](https://cmake.org/cmake/help/latest/command/continue.html)
  - [else](https://cmake.org/cmake/help/latest/command/else.html)
  - [elseif](https://cmake.org/cmake/help/latest/command/elseif.html)
  - [endblock](https://cmake.org/cmake/help/latest/command/endblock.html)
  - [endforeach](https://cmake.org/cmake/help/latest/command/endforeach.html)
  - [endfunction](https://cmake.org/cmake/help/latest/command/endfunction.html)
  - [endif](https://cmake.org/cmake/help/latest/command/endif.html)
  - [endmacro](https://cmake.org/cmake/help/latest/command/endmacro.html)
  - [endwhile](https://cmake.org/cmake/help/latest/command/endwhile.html)
  - [execute_process](https://cmake.org/cmake/help/latest/command/execute_process.html)
  - [file](https://cmake.org/cmake/help/latest/command/file.html)
  - [find_file](https://cmake.org/cmake/help/latest/command/find_file.html)
  - [find_library](https://cmake.org/cmake/help/latest/command/find_library.html)
  - [find_package](https://cmake.org/cmake/help/latest/command/find_package.html)
  - [find_path](https://cmake.org/cmake/help/latest/command/find_path.html)
  - [find_program](https://cmake.org/cmake/help/latest/command/find_program.html)
  - [foreach](https://cmake.org/cmake/help/latest/command/foreach.html)
  - [function](https://cmake.org/cmake/help/latest/command/function.html)
  - [get_cmake_property](https://cmake.org/cmake/help/latest/command/get_cmake_property.html)
  - [get_directory_property](https://cmake.org/cmake/help/latest/command/get_directory_property.html)
  - [get_filename_component](https://cmake.org/cmake/help/latest/command/get_filename_component.html)
  - [get_property](https://cmake.org/cmake/help/latest/command/get_property.html)
  - [if](https://cmake.org/cmake/help/latest/command/if.html)
  - [include](https://cmake.org/cmake/help/latest/command/include.html)
  - [include_guard](https://cmake.org/cmake/help/latest/command/include_guard.html)
  - [list](https://cmake.org/cmake/help/latest/command/list.html)
  - [load_cache](https://cmake.org/cmake/help/latest/command/load_cache.html)
  - [macro](https://cmake.org/cmake/help/latest/command/macro.html)
  - [mark_as_advanced](https://cmake.org/cmake/help/latest/command/mark_as_advanced.html)
  - [math](https://cmake.org/cmake/help/latest/command/math.html)
  - [message](https://cmake.org/cmake/help/latest/command/message.html)
  - [option](https://cmake.org/cmake/help/latest/command/option.html)
  - [return](https://cmake.org/cmake/help/latest/command/return.html)
  - [separate_arguments](https://cmake.org/cmake/help/latest/command/separate_arguments.html)
  - [set](https://cmake.org/cmake/help/latest/command/set.html)
  - [set_directory_properties](https://cmake.org/cmake/help/latest/command/set_directory_properties.html)
  - [set_property](https://cmake.org/cmake/help/latest/command/set_property.html)
  - [site_name](https://cmake.org/cmake/help/latest/command/site_name.html)
  - [string](https://cmake.org/cmake/help/latest/command/string.html)
  - [unset](https://cmake.org/cmake/help/latest/command/unset.html)
  - [variable_watch](https://cmake.org/cmake/help/latest/command/variable_watch.html)
  - [while](https://cmake.org/cmake/help/latest/command/while.html)

- Project Commands
  - [add_compile_definitions](https://cmake.org/cmake/help/latest/command/add_compile_definitions.html)
  - [add_compile_options](https://cmake.org/cmake/help/latest/command/add_compile_options.html)
  - [add_custom_command](https://cmake.org/cmake/help/latest/command/add_custom_command.html)
  - [add_custom_target](https://cmake.org/cmake/help/latest/command/add_custom_target.html)
  - [add_definitions](https://cmake.org/cmake/help/latest/command/add_definitions.html)
  - [add_dependencies](https://cmake.org/cmake/help/latest/command/add_dependencies.html)
  - [add_executable](https://cmake.org/cmake/help/latest/command/add_executable.html)
  - [add_library](https://cmake.org/cmake/help/latest/command/add_library.html)
  - [add_link_options](https://cmake.org/cmake/help/latest/command/add_link_options.html)
  - [add_subdirectory](https://cmake.org/cmake/help/latest/command/add_subdirectory.html)
  - [add_test](https://cmake.org/cmake/help/latest/command/add_test.html)
  - [aux_source_directory](https://cmake.org/cmake/help/latest/command/aux_source_directory.html)
  - [build_command](https://cmake.org/cmake/help/latest/command/build_command.html)
  - [cmake_file_api](https://cmake.org/cmake/help/latest/command/cmake_file_api.html)
  - [cmake_instrumentation](https://cmake.org/cmake/help/latest/command/cmake_instrumentation.html)
  - [create_test_sourcelist](https://cmake.org/cmake/help/latest/command/create_test_sourcelist.html)
  - [define_property](https://cmake.org/cmake/help/latest/command/define_property.html)
  - [enable_language](https://cmake.org/cmake/help/latest/command/enable_language.html)
  - [enable_testing](https://cmake.org/cmake/help/latest/command/enable_testing.html)
  - [export](https://cmake.org/cmake/help/latest/command/export.html)
  - [fltk_wrap_ui](https://cmake.org/cmake/help/latest/command/fltk_wrap_ui.html)
  - [get_source_file_property](https://cmake.org/cmake/help/latest/command/get_source_file_property.html)
  - [get_target_property](https://cmake.org/cmake/help/latest/command/get_target_property.html)
  - [get_test_property](https://cmake.org/cmake/help/latest/command/get_test_property.html)
  - [include_directories](https://cmake.org/cmake/help/latest/command/include_directories.html)
  - [include_external_msproject](https://cmake.org/cmake/help/latest/command/include_external_msproject.html)
  - [include_regular_expression](https://cmake.org/cmake/help/latest/command/include_regular_expression.html)
  - [install](https://cmake.org/cmake/help/latest/command/install.html)
  - [link_directories](https://cmake.org/cmake/help/latest/command/link_directories.html)
  - [link_libraries](https://cmake.org/cmake/help/latest/command/link_libraries.html)
  - [project](https://cmake.org/cmake/help/latest/command/project.html)
  - [remove_definitions](https://cmake.org/cmake/help/latest/command/remove_definitions.html)
  - [set_source_files_properties](https://cmake.org/cmake/help/latest/command/set_source_files_properties.html)
  - [set_target_properties](https://cmake.org/cmake/help/latest/command/set_target_properties.html)
  - [set_tests_properties](https://cmake.org/cmake/help/latest/command/set_tests_properties.html)
  - [source_group](https://cmake.org/cmake/help/latest/command/source_group.html)
  - [target_compile_definitions](https://cmake.org/cmake/help/latest/command/target_compile_definitions.html)
  - [target_compile_features](https://cmake.org/cmake/help/latest/command/target_compile_features.html)
  - [target_compile_options](https://cmake.org/cmake/help/latest/command/target_compile_options.html)
  - [target_include_directories](https://cmake.org/cmake/help/latest/command/target_include_directories.html)
  - [target_link_directories](https://cmake.org/cmake/help/latest/command/target_link_directories.html)
  - [target_link_libraries](https://cmake.org/cmake/help/latest/command/target_link_libraries.html)
  - [target_link_options](https://cmake.org/cmake/help/latest/command/target_link_options.html)
  - [target_precompile_headers](https://cmake.org/cmake/help/latest/command/target_precompile_headers.html)
  - [target_sources](https://cmake.org/cmake/help/latest/command/target_sources.html)
  - [try_compile](https://cmake.org/cmake/help/latest/command/try_compile.html)
  - [try_run](https://cmake.org/cmake/help/latest/command/try_run.html)

- CTest Commands
  - [ctest_build](https://cmake.org/cmake/help/latest/command/ctest_build.html)
  - [ctest_configure](https://cmake.org/cmake/help/latest/command/ctest_configure.html)
  - [ctest_coverage](https://cmake.org/cmake/help/latest/command/ctest_coverage.html)
  - [ctest_empty_binary_directory](https://cmake.org/cmake/help/latest/command/ctest_empty_binary_directory.html)
  - [ctest_memcheck](https://cmake.org/cmake/help/latest/command/ctest_memcheck.html)
  - [ctest_read_custom_files](https://cmake.org/cmake/help/latest/command/ctest_read_custom_files.html)
  - [ctest_run_script](https://cmake.org/cmake/help/latest/command/ctest_run_script.html)
  - [ctest_sleep](https://cmake.org/cmake/help/latest/command/ctest_sleep.html)
  - [ctest_start](https://cmake.org/cmake/help/latest/command/ctest_start.html)
  - [ctest_submit](https://cmake.org/cmake/help/latest/command/ctest_submit.html)
  - [ctest_test](https://cmake.org/cmake/help/latest/command/ctest_test.html)
  - [ctest_update](https://cmake.org/cmake/help/latest/command/ctest_update.html)
  - [ctest_upload](https://cmake.org/cmake/help/latest/command/ctest_upload.html)

- Deprecated Commands

  已被抛弃的命令，在新代码中不要使用下面的命令。
  - [build_name](https://cmake.org/cmake/help/latest/command/build_name.html)
  - [exec_program](https://cmake.org/cmake/help/latest/command/exec_program.html)
  - [export_library_dependencies](https://cmake.org/cmake/help/latest/command/export_library_dependencies.html)
  - [install_files](https://cmake.org/cmake/help/latest/command/install_files.html)
  - [install_programs](https://cmake.org/cmake/help/latest/command/install_programs.html)
  - [install_targets](https://cmake.org/cmake/help/latest/command/install_targets.html)
  - [load_command](https://cmake.org/cmake/help/latest/command/load_command.html)
  - [make_directory](https://cmake.org/cmake/help/latest/command/make_directory.html)
  - [output_required_files](https://cmake.org/cmake/help/latest/command/output_required_files.html)
  - [qt_wrap_cpp](https://cmake.org/cmake/help/latest/command/qt_wrap_cpp.html)
  - [qt_wrap_ui](https://cmake.org/cmake/help/latest/command/qt_wrap_ui.html)
  - [remove](https://cmake.org/cmake/help/latest/command/remove.html)
  - [subdir_depends](https://cmake.org/cmake/help/latest/command/subdir_depends.html)
  - [subdirs](https://cmake.org/cmake/help/latest/command/subdirs.html)
  - [use_mangled_mesa](https://cmake.org/cmake/help/latest/command/use_mangled_mesa.html)
  - [utility_source](https://cmake.org/cmake/help/latest/command/utility_source.html)
  - [variable_requires](https://cmake.org/cmake/help/latest/command/variable_requires.html)
  - [write_file](https://cmake.org/cmake/help/latest/command/write_file.html)

## 3.1. Scripting Commands

###  3.1.1. cmake_minimum_required

```cmake
# 设置最低CMake版本要求
cmake_minimum_required(VERSION 3.12)
```



### 3.1.2. messge

在 CMake 中，`message()` 是一个**内置命令（command）**，用于在配置阶段向用户输出信息、警告或错误。它常用于调试脚本、提示用户、或强制终止构建流程。

**基本语法**

```cmake
message([<mode>] "message text")
```

其中 `<mode>` 是可选的，用于指定消息的类型（级别）。如果不指定，默认为 `STATUS`。

消息模式（mode）详解。

| 模式             | 行为说明                                                     |
| ---------------- | ------------------------------------------------------------ |
| (无) 或 `STATUS` | 输出普通状态信息（绿色前缀 `--`），不会中断配置。            |
| `WARNING`        | 输出警告（黄色），继续配置。                                 |
| `AUTHOR_WARNING` | 面向项目作者的警告（通常用于 CMake 脚本开发者），继续配置。  |
| `SEND_ERROR`     | 报错但不立即停止配置；CMake 会在当前处理完成后退出（非零状态）。 |
| `FATAL_ERROR`    | 立即终止 CMake 配置过程，并返回错误（最常用在严重错误时）。  |
| `DEPRECATION`    | 显示弃用警告（行为类似 `AUTHOR_WARNING`，但语义更明确）。    |
| `NOTICE`         | 类似 `STATUS`，但用于更正式的通知（CMake 3.17+）。           |
| `VERBOSE`        | 仅在启用详细输出（如 `-Wdev` 或高日志级别）时显示。          |
| `DEBUG`          | 仅在调试模式下显示（需设置 `CMAKE_DEBUG_OUTPUT=ON` 等）。    |

💡 提示：`STATUS` 是默认模式，所以 `message("Hello")` 等价于 `message(STATUS "Hello")`。

**示例**

```cmake
# 打印环境变量 $PATH
message($ENV{PATH})

# 普通状态信息（最常见）
message(STATUS "Configuring MyProject...")
message("Found Python: ${Python_EXECUTABLE}")  # 等价于 STATUS

# 警告
message(WARNING "This feature is experimental!")

# 调试
message(STATUS "CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}")
```

**注意事项**

1. 变量展开：

   CMake 会在 `message()` 中自动展开  `${VAR}` 变量。

   ```cmake
   set(NAME "Alice")
   message("Hello, ${NAME}!")  # 输出：Hello, Alice!
   ```

2. 换行与多行：
   `message()` 默认不支持直接换行，但可通过多次调用或使用 `configure_file()` 等方式间接实现。

3. 性能影响：
   `message()` 在 configure 阶段执行，不影响编译速度，但过多输出可能干扰用户。

4. 颜色输出：
   终端支持时，不同模式会自动着色（如 `FATAL_ERROR` 红色，`WARNING` 黄色）。

### 3.1.3. set

 `set()` 是变量赋值命令。在一条 set 命令中可以同时设置多个变量。 

**功能**

- 创建或修改一个变量的值。
- 可用于普通变量、缓存变量（cache variables）、环境变量等。

**基本语法**

```cmake
set(<variable> <value> [CACHE <type> <docstring> [FORCE]])
```

**示例**

```cmake
set(MY_VAR "Hello")                    # 普通变量
set(CMAKE_CXX_STANDARD 17)             # 设置标准（也是普通变量）
set(MY_CACHE_VAR "value" CACHE STRING "A cached variable")  # 缓存变量
```

编译器相关设置

```cmake
# 添加c++11标准支持
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")    

# 默认c编译器
set(CMAKE_C_COMPILER "gcc.exe")

# 默认c++编译器
set(CMAKE_CXX_COMPILER "g++.exe")

# 设置编译类型为 debug
set(CMAKE_BUILD_TYPE Debug)

# 设置编译类型为 release
set(CMAKE_BUILD_TYPE Release)
```

GDB 调试设置

```cmake
# Debug模式 选项: Release Debug MinSizeRel RelWithDebInfo
set(CMAKE_BUILD_TYPE "Debug")

# debug模式下 gdb相关选项
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")  

# release模式下 gdb相关选项
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")  

# 开启调试 出现问题时开启
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
```

可执行文件

```cmake
# 设置可执行文件输出的目录
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)   
```

### 3.1.4. unset

删除变量命令

**功能**

- **完全移除**一个变量（包括其值和缓存条目），使其不再存在。
- 如果变量是缓存变量，`unset()` 会同时从缓存中删除它。

**语法**

```cmake
unset(<variable> [CACHE])
```

**示例**

```cmake
set(MY_VAR "test")
unset(MY_var)        # 删除普通变量（注意：CMake 变量名大小写敏感！）

set(MY_CACHE_VAR "val" CACHE STRING "desc")
unset(MY_CACHE_VAR CACHE)  # 显式删除缓存变量
```

⚠️ 注意：`unset(VAR)` 不加 `CACHE` 时，只删除普通作用域中的变量；如果该变量同时存在于缓存中，缓存副本仍然保留。若要彻底删除缓存变量，需使用 `unset(VAR CACHE)`。

###  3.1.5. if/elseif

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

### 3.1.6. file

在 CMake 中，`file()` 是一个**功能极其丰富**的命令，用于执行各种与**文件系统、文件内容、路径操作**相关的任务。它是 CMake 脚本中处理文件的核心工具之一。涵盖：

- 读写文件内容
- 文件系统操作（复制、删除、创建目录）
- 路径处理与查询
- 网络下载
- 内容生成

**使用原则：**

> ✨ **优先使用 `target_\*` 和 `install()` 管理构建产物；仅在必要时用 `file()` 处理 configure 阶段的文件逻辑。**

**语法：**

```cmake
file(<MODE> [arguments...])
```

其中 `<MODE>` 决定了 `file()` 的具体行为。不同模式支持不同的参数。

**注意事项**

| 事项             | 说明                                                         |
| ---------------- | ------------------------------------------------------------ |
| 执行时机         | 大多数 `file()` 操作在 configure 阶段（运行 `cmake` 时）执行，不是 `make` 时。 |
| 路径分隔符       | CMake 内部统一使用 `/`，即使在 Windows 上也有效。            |
| 变量展开         | 在 `WRITE`/`READ` 等内容操作中，`${VAR}` 会被展开（除非用 `[[ ]]` 或转义）。 |
| 避免 GLOB 源文件 | 自动收集源文件会导致构建系统无法感知新增文件，需手动重跑 CMake。 |

### 3.1.7. option

`option()` 是一个用于定义**可配置的布尔选项**（开关）的命令。它通常用于让用户在配置项目时启用或禁用某些功能、依赖项或构建行为。

**语法**

```cmake
option(<option_variable> "描述信息" [ON|OFF])
```

- **`<option_variable>`**：选项的变量名（通常使用大写命名，如 `BUILD_TESTS`）。
- **`"描述信息"`**：对该选项用途的简要说明，会在 CMake GUI 或 `ccmake` 等工具中显示。
- **`[ON|OFF]`**（可选）：默认值。如果不指定，默认为 `OFF`。

**示例：**

```cmake
option(BUILD_TESTS "Build the test suite" ON)
option(USE_OPENMP "Enable OpenMP support" OFF)
```

用户在配置项目时可以覆盖这些默认值：

```bash
cmake -DBUILD_TESTS=OFF -DUSE_OPENMP=ON ..
```

在 CMakeLists.txt 中，你可以根据这些选项控制构建逻辑：

```cmake
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

if(USE_OPENMP)
    find_package(OpenMP REQUIRED)
    target_link_libraries(myapp PRIVATE OpenMP::OpenMP_CXX)
endif()
```

**注意事项**

1. **作用域**：`option()` 定义的是**缓存变量**（cache variable），因此它的值会被保存在 `CMakeCache.txt` 中。这意味着即使你修改了 CMakeLists.txt 中的默认值，除非清除缓存或显式覆盖，否则旧值仍会保留。
2. **类型安全**：`option()` 只能设置布尔值（`ON`/`OFF`、`TRUE`/`FALSE`、`1`/`0` 等等效）。不要用它来设置字符串或路径。
3. 与普通变量的区别：
   - 普通变量（如 `set(MY_VAR value)`）不会出现在缓存中，也不会被用户直接配置。
   - `option()` 创建的变量会出现在 CMake 配置界面中，便于用户交互。

------

**实际应用场景**

- 控制是否构建示例程序、测试、文档。
- 启用/禁用可选依赖（如 `ENABLE_SSL`、`WITH_PYTHON_BINDINGS`）。
- 调试/发布模式开关（虽然通常用 `CMAKE_BUILD_TYPE`，但也可自定义）。

## 3.2. Project Commands

### 3.2.1. project

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

### 3.2.2. add_definitions

 设置字符集

```cmake
add_definitions(-DUNICODE -D_UTF-8) 
```

### 3.2.3. add_compile_options

添加编译参数

```cmake
语法：add_compile_options(<option> ...)

# 添加编译参数 -Wall std=c++11 -O3
add_compile_options(-Wall std=c++11 -O3)
```

### 3.2.4. add_library

生成一个可执行目标，可执行目标可以是 ：静态库（STATIC）、动态库（SHARED）或对象库（OBJECT）。默认是生成静态库（STATIC library）。

**语法**

```cmake
add_library(libname [SHARED | STATIC | MODULE] [EXCULD_FROM_ALL] src1 src2 ...)
```

**示例**

```cmake
# 通过变量 SRC 生成 libtest.so 共享库，生成的时候会加上 lib 前缀和 .so 后缀
add_library(test SHARED ${SRC})
```

### 3.2.5. add_executable

`add_executable` 是 CMake 中用于定义可执行文件（executable）目标的核心命令。它告诉 CMake：**“请从指定的源文件编译并链接出一个可执行程序”**。

语法

```cmake
add_executable(<name> [source1] [source2] ...)
```

- `<name>`：生成的可执行文件名称（不带路径，CMake 会根据平台自动加 `.exe` 等后缀）。
- `[source...]`：构成该程序的源文件列表（如 `.cpp`, `.c`, `.cc`, `.f90` 等）。

**示例：**

```cmake
add_executable(hello main.cpp utils.cpp)
```

这会将 `main.cpp` 和 `utils.cpp` 编译并链接成名为 `hello`（在 Windows 上是 `hello.exe`）的可执行文件。

**注意点：**

- **目标名称必须唯一**。
  在整个 CMake 项目中（包括子目录），所有 `add_executable` 和 `add_library` 的目标名不能重复。

- **必须配合其他命令使用**
  `add_executable` 只定义了“要构建什么”，但通常还需：

  - `target_include_directories()`：指定头文件路径
  - `target_link_libraries()`：链接库
  - `target_compile_definitions()`：定义宏等

- **可执行文件名 ≠ 目标名（可通过 OUTPUT_NAME 修改）**。
  默认情况下，生成的可执行文件名与目标名相同，但你可以修改：

  ```cmake
  add_executable(my_target main.cpp)
  set_target_properties(my_target PROPERTIES OUTPUT_NAME "my_real_program")
  ```

### 3.2.6. add_subdirectory

向工程中添加存放源文件的子目录，作为可选项，可指定二进制文件或二进制文件存放的位置。

**语法**

```cmake
add_subdirectory(source_dir [binary_dir] [EXCLUDE_FROM_ALL])
```

**示例**

```cmake
# 工程中添加 google 子目录
add_subdirectory(google)
```

### 3.2.7. include_directories

将指定路径添加到**所有后续目标**的头文件搜索路径中（即 `-I` 编译选项）。

**语法**

```cmake
include_directories([AFTER|BEFORE] [SYSTEM] dir1 [dir2 ...])
```

**特点：**

- **全局作用域**：影响当前 CMakeLists.txt 及其子目录中所有后续定义的目标（如可执行文件、库）。
- 不推荐在现代 CMake（3.0+）中使用，因为它缺乏目标粒度控制，容易造成污染。

**示例**：

```cmake
include_directories(/usr/local/include)
add_executable(myapp main.cpp)  # myapp 会包含 /usr/local/include 作为头文件路径
```

### 3.2.8. link_directories

将指定库路径添加到**所有后续目标**的库搜索路径中（即 `-L` 链接选项）。

**语法：**

```cmake
link_directories(dir1 [dir2 ...])
```

**特点：**

- **全局作用域**：影响当前 CMakeLists.txt 及其子目录中所有后续目标。
- 通常不推荐使用，因为现代 CMake 更倾向于使用 `find_package()` 或直接指定库的完整路径。

**示例：**

```cmake
link_directories(/opt/mylib/lib)
target_link_libraries(myapp mylib)  # 链接器会在 /opt/mylib/lib 中找 libmylib.so
```

### 3.2.9. target_sources

`target_sources()` 是现代 CMake（3.1+，推荐 3.13+）中**管理目标源文件的首选方式**，它比传统的 `set(SRCS ...)` + `add_executable(... ${SRCS})` 更清晰、模块化，并支持作用域控制。

**语法：**

```cmake
target_sources(<target>
  <PRIVATE|PUBLIC|INTERFACE>
  [items...]
)
```

- `<target>`：已通过 `add_executable()`、`add_library()` 等创建的目标名称。
- `<PRIVATE|PUBLIC|INTERFACE>`：指定这些源文件的“可见性”（对依赖该目标的其他目标是否可见）。
- `[items...]`：源文件列表（`.cpp`, `.c`, `.h`, `.cu` 等），支持相对路径或绝对路径。

> 💡 注意：`target_sources()` **必须在目标创建之后调用**。

**可见性解释**

| 关键字      | 含义                                                         |
| ----------- | ------------------------------------------------------------ |
| `PRIVATE`   | 源文件仅用于构建当前目标本身，不暴露给依赖此目标的其他目标。适用于实现文件（如 `.cpp`）。 |
| `PUBLIC`    | 源文件既用于构建当前目标，也暴露给依赖者。通常用于头文件（尤其是模板或 inline 函数定义在 `.h` 中的情况）。 |
| `INTERFACE` | 源文件不参与当前目标的构建，但会暴露给依赖者。极少用于源文件，更多用于接口库（`INTERFACE` 库）。 |

📌 对于**普通可执行文件或静态/动态库**，绝大多数源文件（`.cpp`）应使用 `PRIVATE`；头文件一般不需要列在 `target_sources` 中（除非是自动生成的或需要安装），但若要包含，通常用 `PUBLIC` 或 `PRIVATE`。

✅ 示例 1：构建可执行文件（推荐写法）

```cmake
# 先创建空目标（CMake 3.11+ 支持）
add_executable(myapp "")

# 添加源文件
target_sources(myapp PRIVATE
    src/main.cpp
    src/utils.cpp
    src/logger.cpp
)

# 可以多次调用 target_sources 扩展源文件
target_sources(myapp PRIVATE
    src/network.cpp
)
```

------

**✅ 示例 2：构建库并导出头文件（用于安装或导出）**

```cmake
add_library(mymath "")

target_sources(mymath
  PRIVATE
    src/add.cpp
    src/mul.cpp
  PUBLIC
    include/mymath/add.h
    include/mymath/mul.h
)

# 设置头文件目录，让使用者能 #include <mymath/add.h>
target_include_directories(mymath
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
```

**优点**

1. **作用域明确**：通过 `PRIVATE/PUBLIC` 控制依赖传播，符合现代 CMake 的“目标中心”思想。

2. **可组合性强**：可在多个 `CMakeLists.txt` 中逐步添加源文件，适合大型项目。

3. **与 `target_include_directories`、`target_link_libraries` 风格统一**，形成一致的 API。

4. **支持生成器表达式**（如 `$<CONFIG>`），可按构建类型选择不同源文件：

   ```cmake
   target_sources(myapp PRIVATE
       $<$<CONFIG:Debug>:debug_utils.cpp>
   )
   ```

   



### 3.2.10. target_include_directories

用于替代 `include_directories()`。为**特定目标**设置头文件搜索路径，并可控制可见性（私有、公有、接口）。

**语法**：

```cmake
target_include_directories(target
    [PRIVATE|PUBLIC|INTERFACE] dirs...
    ...
)
```

**可见性说明：**

- `PRIVATE`：仅本目标编译时使用。
- `PUBLIC`：本目标使用 + 链接到本目标的其他目标也会继承。
- `INTERFACE`：仅对链接到本目标的其他目标生效（本目标不使用）。

**优点：**

- 精确控制依赖传播。
- 符合现代 CMake 的“基于目标”的最佳实践。

**示例：**

```cmake
add_library(mymath math.cpp)
target_include_directories(mymath PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(myapp main.cpp)
target_link_libraries(myapp mymath)  # 自动继承 mymath 的 PUBLIC include 路径
```

### 3.2.11. target_link_libraries

指定**某个目标**需要链接哪些库，并可控制依赖传播（PRIVATE / PUBLIC / INTERFACE）。

语法

```cmake
target_link_libraries(target
    [PRIVATE|PUBLIC|INTERFACE] item1 [item2 ...]
)
```

其中 `item` 可以是：

- 库名（如 `pthread`）
- 其他 CMake 目标（如 `mymath`）
- 完整路径的 `.so`/`.a` 文件
- 导入的库（通过 `find_package` 得到）

**可见性说明（与** `target_include_directories` **类似）：**

- `PRIVATE`：仅本目标链接该库。
- `PUBLIC`：本目标链接 + 链接到本目标的其他目标也需链接该库。
- `INTERFACE`：仅对链接到本目标的其他目标生效。

示例

```cmake
find_package(Threads REQUIRED)
add_executable(myapp main.cpp)
target_link_libraries(myapp PRIVATE Threads::Threads)
```

`target` 目标名字必须通过 `add_executable()` 或 `add_library()` 命令创建的，不能是一个别名。

### 3.2.12. install

CMake 中的 `install()` 命令用于定义在执行安装（`make install` 或 `ninja install` 等）时，哪些文件、目录、目标（targets）、脚本等应该被复制到指定的目标位置。它是构建可分发软件包（如 RPM、DEB、MSI 或 tar.gz）的关键部分。

---

#### 3.2.12.1. 语法

`install()` 命令有多种签名（signatures），对应不同类型的安装内容：

```cmake
install(TARGETS targets... [EXPORT <export-name>] ...)
install(FILES files... ...)
install(DIRECTORY dirs... ...)
install(PROGRAMS programs... ...)
install(SCRIPT script-file)
install(CODE "code")
install(EXPORT <export-name> ...)
```

#### 3.2.12.2. 安装前缀(prefix)

默认安装路径由 `CMAKE_INSTALL_PREFIX` 控制。安装的时候可以指定绝对路径，也可以指定相对路径。其中，使用相对路径时，

`cmake_install_prefix` 默认安装路径在 `/usr/local`；自己指定文件安装路径：`cmake_install_prefix=/usr`

```cmake
# 安装文件到某个目录下
install(directory doc/ destation share/doc/cmake)
```

安装的文件后面是否带有 `/`，安装时有很大的区别：

- `doc/` 文件后带有 `/` 表示将 `doc` 路径下的所有文件安装到 `/usr/local/share/doc/cmake` 路径下。
- `doc` 文件后不带 `/` 表示将 `doc` 整个文件安装到 `/usr/local/share/doc/cmake` 路径下。

外部使用 CMake 编译时，可用下面的命令改变安装路径。

```cmake
cmake -DCMAKE_INSTALL_PREFIX=/opt/myapp ..
```

#### 3.2.12.3. 安装目标(TARGETS)

这是最常用的用法，用于安装编译生成的可执行文件、库等。

**语法**：
```cmake
install(TARGETSJ targets...
        [ARCHIVE DESTINATION <dir> [PERMISSIONS ...] [CONFIGURATIONS [Debug|Release|...]]]
        [LIBRARY DESTINATION <dir> ...]
        [RUNTIME DESTINATION <dir> ...]
        [OBJECTS DESTINATION <dir> ...]
        [FRAMEWORK DESTINATION <dir> ...]
        [BUNDLE DESTINATION <dir> ...]
        [PUBLIC_HEADER DESTINATION <dir> ...]
        [PRIVATE_HEADER DESTINATION <dir> ...]
        [RESOURCE DESTINATION <dir> ...]
        [OPTIONAL]
        [EXCLUDE_FROM_ALL]
        [NAMELINK_COMPONENT <component>]
        [NAMELINK_ONLY | NAMELINK_SKIP]
        [INCLUDES DESTINATION [<dir> ...]]
)
```

**示例**：
```cmake
add_library(mylib SHARED src/mylib.cpp)
add_executable(myapp src/main.cpp)

# 安装 mylib 到 lib 目录，myapp 到 bin 目录
install(TARGETS mylib myapp
        LIBRARY DESTINATION lib    # Linux/macOS 共享库
        ARCHIVE DESTINATION lib    # 静态库
        RUNTIME DESTINATION bin    # 可执行文件（Windows/Linux）
)
```

> 注意：不同平台对目标类型使用不同的关键字：
>
> - **Linux/Unix**：共享库 → `LIBRARY`；静态库 → `ARCHIVE`；可执行文件 → `RUNTIME`
> - **Windows**：DLL → `RUNTIME`；.lib 导入库 → `ARCHIVE`
> - **macOS Frameworks**：使用 `FRAMEWORK`

#### 3.2.12.4. 安装普通文件(FILES)

用于安装头文件、配置文件、文档等。

**语法：**

```cmake
install(FILES files...
        DESTINATION <dir>
        [PERMISSIONS permissions...]
        [CONFIGURATIONS [Debug|Release|...]]
        [RENAME <name>]
        [OPTIONAL]
)
```

**示例：**

```cmake
install(FILES include/mylib.h DESTINATION include)
install(FILES README.md LICENSE DESTINATION share/doc/myproject)
```

**Permissions(权限)**

`PERMISSIONS` 是一个可选参数，用于**显式设置安装后文件或目录的权限（file permissions）**，尤其在 **Unix/Linux/macOS** 系统上非常重要。

```cmake
install(FILES myscript.sh
        DESTINATION bin
        PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
                    GROUP_READ GROUP_EXECUTE
                    WORLD_READ WORLD_EXECUTE)
```

这行代码的意思是：安装 `my_script.sh` 到 `bin` 目录，并设置其权限为：

- 所有者（Owner）：可读、可写、可执行（`rwx` → `7`）
- 所属组（Group）：可读、可执行（`r-x` → `5`）
- 其他人（World）：可读、可执行（`r-x` → `5`）

最终权限相当于 `chmod 755 my_script.sh`。

🔐 **常见权限常量（CMake 内置）**

| CMake 权限标志  | 对应 Unix 权限位   | 含义         |
| --------------- | ------------------ | ------------ |
| `OWNER_READ`    | `r--------` (400)  | 所有者可读   |
| `OWNER_WRITE`   | `-w-------` (200)  | 所有者可写   |
| `OWNER_EXECUTE` | `--x------` (100)  | 所有者可执行 |
| `GROUP_READ`    | `---r-----` (040)  | 组可读       |
| `GROUP_WRITE`   | `----w----` (020)  | 组可写       |
| `GROUP_EXECUTE` | `-----x----` (010) | 组可执行     |
| `WORLD_READ`    | `------r--` (004)  | 其他人可读   |
| `WORLD_WRITE`   | `-------w-` (002)  | 其他人可写   |
| `WORLD_EXECUTE` | `--------x` (001)  | 其他人可执行 |

> 💡 这些常量可以任意组合，CMake 会自动计算出对应的八进制权限值。

#### 3.2.12.5. 安装目录(DIRECTORY)

递归或非递归地安装整个目录结构。

`DIRECTORY` 和 `DESTINATION` 是 **安装（install）命令** 中常用的两个关键字，用于指定将哪些目录内容安装到目标路径。它们通常一起出现在 `install(DIRECTORY ...)` 命令中。

语法：

```cmake
install(DIRECTORY dirs...
        DESTINATION <dir>
        [FILE_PERMISSIONS ...]
        [DIRECTORY_PERMISSIONS ...]
        [USE_SOURCE_PERMISSIONS]
        [CONFIGURATIONS [Debug|Release|...]]
        [FILES_MATCHING]
        [PATTERN <pattern> | REGEX <regex>]
        [EXCLUDE]
        [PERMISSIONS ...]
)
```

示例：

```cmake
# 将 config/ 目录完整复制到 ${CMAKE_INSTALL_PREFIX}/etc/myapp/
install(DIRECTORY config/ DESTINATION etc/myapp)
```

> 注意：目录名末尾是否有 `/` 很重要：
>
> - `config/`：只复制内容
> - `config`：复制整个目录（变成 `etc/myapp/config/...`）

#### 3.2.12.6. 安装程序(PROGRAMS)

与 `FILES` 类似，但默认设置可执行权限（主要用于脚本）。

```cmake
install(PROGRAMS scripts/myscript.sh DESTINATION bin)
```

#### 3.2.12.7. 安装脚本或代码(SCRIPT / CODE)

- `SCRIPT`：执行一个 CMake 脚本文件（在安装时运行）
- `CODE`：直接内联 CMake 命令

```cmake
install(SCRIPT cmake/post_install.cmake)
install(CODE "message(\"Installation complete!\")")
```

> 常用于注册服务、创建符号链接、设置环境变量等。

#### 3.2.12.8. 导出目标供其他项目使用（EXPORT）

配合 `install(EXPORT ...)` 和 `install(TARGETS ... EXPORT ...)`，可生成 CMake 配置文件，使其他项目通过 `find_package()` 使用你的库。

示例：

```cmake
install(TARGETS mylib
        EXPORT MyLibTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        PUBLIC_HEADER DESTINATION include
)

install(EXPORT MyLibTargets
        FILE MyLibTargets.cmake
        NAMESPACE MyLib::
        DESTINATION lib/cmake/MyLib
)
```

然后生成 `MyLibConfig.cmake` 文件（通常手动写或用 `configure_package_config_file`），用户就可以：

```cmake
find_package(MyLib REQUIRED)
target_link_libraries(myapp MyLib::mylib)
```

#### 3.2.12.9. 组件(COMPONENT)

支持按组件安装（用于打包系统）：

```cmake
install(FILES doc.pdf DESTINATION share/doc COMPONENT documentation)
install(TARGETS myapp DESTINATION bin COMPONENT runtime)
```

然后可以只安装特定组件：

```bash
cmake --install . --component runtime
```

#### 3.2.12.10. 示例

```cmake
cmake_minimum_required(VERSION 3.14)
project(MyProject)

add_library(mylib SHARED src/mylib.cpp)
add_executable(myapp src/main.cpp)
target_link_libraries(myapp mylib)

# 安装库和可执行文件
install(TARGETS mylib myapp
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
)

# 安装头文件
install(FILES include/mylib.h DESTINATION include)

# 安装配置文件
install(DIRECTORY config/ DESTINATION etc/myapp)

# 导出目标
install(TARGETS mylib
        EXPORT MyLibTargets
        PUBLIC_HEADER DESTINATION include
)
install(EXPORT MyLibTargets
        FILE MyLibTargets.cmake
        NAMESPACE MyLib::
        DESTINATION lib/cmake/MyLib
)
```

# 4. Build

## 4.1. 内部构建

**不推荐使用。**

内部构建会在同级目录产生一大堆中间文件，并放到和源工程同级的位置，但这些中间文件并不是我们所需要的，放在一起使工程显得杂乱无章，结构不清晰。

```cmake
# 当前目录下编译本目录的 CMakeLists.txt 文件，生成 Makefile 和其它文件
cmake .

# 当前路径执行 make 命令，生成  target
make
```

## 4.2. 外部构建

推荐使用。将编译输出的文件与源文件放到不同的目录中。

第一种方式：先创建 build 文件，然后进入build 目录，在目录里面执行 make命令。

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

第二种方式，直接在 `CMakeLists.txt` 同一级目录下执行下面的命令，不用先创建目录再进入 build 目录，省略了步骤。

```bash
# 创建 build 文件，并在 build文件中生成 makefile 和其它编译的文件
cmake -B build

# 生成项目
cmake --build build
```



# 5. Usage

**小贴士**

- 🌟 **现代 CMake（≥3.0）** 推荐使用 `target_*` 命令。（如 `target_include_directories`、`target_link_libraries`），而不是全局命令（如 `include_directories`、`link_libraries`），以实现更好的封装性和可组合性。
- 使用 `PRIVATE`、`PUBLIC`、`INTERFACE` 明确依赖传递关系。

示例 `CMakeLists.txt`

```cmake
# 指定所需的最低 CMake 版本
cmake_minimum_required(VERSION 3.x)

# 定义项目名称、版本和使用的语言
project(MyProject VERSION 1.0 LANGUAGES CXX)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找已安装的包，并导入 target
find_package(PkgName REQUIRED)

# 手动查找特定库文件（较少用，推荐用 find_package + imported target）。
find_library(VAR_NAME lib_name PATHS /usr/local/lib)

# 添加子目录
add_subdirectory(subdir)

# 创建静态库、动态库、OBJECT
add_library(target_name STATIC|SHARED|OBJECT source1.cpp ...)

# 创建一个可执行文件目标
add_executable(target_name source1.cpp source2.cpp)

# 头文件搜索路径
target_include_directories(my_app PRIVATE include)

# 添加源文件
target_sources(<target>
  <PRIVATE|PUBLIC|INTERFACE>
  [items...]
)

# 链接依赖库
target_link_libraries(target_name PRIVATE|PUBLIC|INTERFACE lib1 lib2)

# 为目标添加编译器选项（如警告标志）
target_compile_options(my_app PRIVATE -Wall -Wextra)

# 为目标定义预处理器宏。
target_compile_definitions(target_name PRIVATE DEBUG=1)
```



# 6. References

- CMake 官网: http://www.cmake.org/
- CMake Reference Documentation: https://cmake.org/cmake/help/latest/index.html
- CMake 入门实战: https://www.hahack.com/codes/cmake/
- CLoin 与 CMake详细教程: https://www.jetbrains.com/help/clion/quick-tutorial-on-configuring-clion-on-windows.html
- CMake 添加编译选项: https://www.cnblogs.com/standardzero/p/10798603.html
