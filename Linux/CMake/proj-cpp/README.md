<!--
 * @Author: JohnJeep
 * @Date: 2024-10-10 15:31:51
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-01-01 17:16:57
 * @Description: 
 * Copyright (c) 2026 by John Jeep, All Rights Reserved. 
-->

结构清晰、可运行的 Linux 下 C++20 工程 CMake 构建方案，满足以下要求：
- 使用 CMake 构建；
- 采用 C++20 标准；
- 工程包含多个子模块（每个子模块编译成动态库）；
- 主程序依赖这些子模块动态库、本工程头文件、以及外部第三方库（含头文件和 .so 动态库）；
- 第三方库路径可配置（例如通过 CMAKE_PREFIX_PATH 或自定义变量）。


项目结构

```
my_project/
├── CMakeLists.txt              # 根CMakeLists
├── include/                    # 工程公共头文件
│   └── common/
│       └── utils.hpp
├── third_party/               # 第三方库（头文件和so文件）
│   ├── include/
│   │   ├── external/
│   │   │   └── lib1.hpp
│   │   └── another/
│   │       └── lib2.hpp
│   └── lib/
│       ├── libexternal.so
│       └── libanother.so
├── module_a/                  # 模块A
│   ├── CMakeLists.txt
│   ├── include/
│   │   └── module_a/
│   │       └── a_utils.hpp
│   └── src/
│       ├── a_utils.cpp
│       └── a_class.cpp
├── module_b/                  # 模块B
│   ├── CMakeLists.txt
│   ├── include/
│   │   └── module_b/
│   │       └── b_utils.hpp
│   └── src/
│       ├── b_utils.cpp
│       ├── b_class.cpp
│       └── b_interface.cpp
└── app/                       # 主程序
    ├── CMakeLists.txt
    └── src/
        └── main.cpp
```
