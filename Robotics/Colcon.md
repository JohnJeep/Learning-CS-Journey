<!--
 * @Author: JohnJeep
 * @Date: 2025-12-07 23:39:49
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-01-31 16:34:24
 * @Description: Colcon Usage
 * Copyright (c) 2026 by John Jeep, All Rights Reserved. 
-->

- [1. Introduction](#1-introduction)
- [2. create](#2-create)
- [3. build](#3-build)
- [4. test](#4-test)
- [5. Env](#5-env)
- [6. References](#6-references)

# 1. Introduction

Colcon 是一个用于构建和管理多个软件包的命令行工具，广泛应用于 ROS 2（Robot Operating System 2）生态系统中。它旨在简化复杂项目的构建过程，支持多种编程语言和构建系统，如 CMake 和 Python。

Colcon 提供了一种统一的方式来处理依赖关系、构建顺序和安装过程，使开发者能够更高效地管理大型代码库。

# 2. create

```bash
# 创建新包（Python）
ros2 pkg create my_python_pkg --build-type ament_python

# 创建新包（C++）
ros2 pkg create my_cpp_pkg --build-type ament_cmake


```

# 3. build

```bash
# 构建新包
colcon build --packages-select my_python_pkg

# 构建并安装到 install 目录
colcon build --symlink-install

# 构建时允许并行处理
colcon build --parallel-workers 8

# 只构建有变化的包
colcon build

# 清理构建缓存
colcon build --cmake-clean-cache

# 构建指定类型的包（CMake 或 Python）
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon build --packages-select my_package --cmake-args -DCMAKE_BUILD_TYPE=Debug

# 继续构建，即使某些包失败
colcon build --continue-on-error
```

# 4. test

```bash
# 构建并运行测试
colcon test

# 运行特定包的测试
colcon test --packages-select package_name

# 查看测试结果
colcon test-result --verbose
```

# 5. Env

构建后需要 source 安装目录的 setup 文件：

```bash
# 在 ~/.bashrc 中添加
source ~/ros2_ws/install/setup.bash

# 或手动 source
source install/local_setup.bash  # 只 source 当前工作空间
source install/setup.bash        # source 所有工作空间
```

# 6. References

- [Colcon 官方文档](https://colcon.readthedocs.io/en/released/)
