<!--
 * @Author: JohnJeep
 * @Date: 2026-03-09 18:54:36
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-03-09 19:10:28
 * @Description: ROS2 packages Usage
 * Copyright (c) 2026 by John Jeep, All Rights Reserved. 
-->

- [1. Introduction](#1-introduction)
- [2. rcl](#2-rcl)
- [3. rclcpp](#3-rclcpp)
  - [3.1. rclcpp::spin(node)](#31-rclcppspinnode)
  - [3.2. internal fundamental](#32-internal-fundamental)
  - [3.3. 什么时候 spin 会结束？](#33-什么时候-spin-会结束)
  - [3.4. spin 的几种用法](#34-spin-的几种用法)
  - [3.5. spin 选择建议](#35-spin-选择建议)
- [4. rclpy](#4-rclpy)
- [5. rmw](#5-rmw)
- [6. node](#6-node)
- [7. timer](#7-timer)
- [8. parameter](#8-parameter)
- [9. publisher](#9-publisher)
- [10. sub](#10-sub)
- [12. rosidl](#12-rosidl)
- [13. urdf](#13-urdf)
- [14. References](#14-references)


# 1. Introduction

在 ROS 2（Robot Operating System 2）中，“标准包”通常指由官方维护、为机器人开发提供基础功能的 **核心功能包（Core Packages）** 或 **标准接口包（Common Interfaces）**。

**标准消息接口包 (Common Interfaces)**

这些包定义了通用的数据结构，使不同开发者的节点能够无缝通信。

- **`std_msgs`**: 提供基础数据类型，如 `String`, `Int32`, `Float64`, `Bool` 等。
- **`geometry_msgs`**: 定义常见的几何图形消息，如点（`Point`）、向量（`Vector3`）、位姿（`Pose`）和速度指令（`Twist`）。
- **`sensor_msgs`**: 涵盖各种传感器数据格式，如激光雷达（`LaserScan`）、图像（`Image`）、点云（`PointCloud2`）和惯性测量单元（`Imu`）。
- **`nav_msgs`**: 用于导航的数据，如地图（`OccupancyGrid`）和路径（`Path`）。

**核心系统工具包 (Core Tooling)**

- **`rclcpp` / `rclpy`**: 分别为 C++ 和 Python 提供的客户端库，是编写 ROS 2 节点的标准入口。
- **`ament_cmake` / `ament_python`**: 用于包构建和管理的标准构建系统。
- **`launch`**: 用于启动多个节点并管理其生命周期的标准启动系统。
- **`ros2cli`**: 提供命令行工具（如 `ros2 node`, `ros2 topic`, `ros2 pkg`）的标准包。

**关键中间件与底层包**

- **`rmw` (ROS Middleware)**: 定义了 ROS 2 与底层 DDS 通信的标准接口。
- **`rcutils`**: 包含 C 语言编写的底层跨平台工具函数。

**功能包的标准结构**

一个标准的 ROS 2 软件包含有特定的文件布局：

- **`package.xml`**: 包含包的元数据（名称、版本、作者、许可证）以及对其他包的依赖声明。
- **`CMakeLists.txt`** (C++): C++ 包的编译规则文件。
- **`setup.py`** (Python): Python 包的分发与安装配置文件


# 2. rcl



# 3. rclcpp



## 3.1. rclcpp::spin(node)

`rclcpp::spin(node)` 是ROS2节点运行的核心机制
- 🎯 **作用**：保持节点活跃，处理所有异步事件
- ⚡ **必要性**：没有它，所有回调函数都不会执行
- 🔄 **行为**：阻塞当前线程，持续处理事件
- 🛑 **退出**：通过信号或显式关闭来停止

理解 `spin` 是编写功能完整的ROS2节点的关键！这是节点从"静态代码"变成"动态系统"的魔法所在。 ✨


## 3.2. internal fundamental

`rclcpp::spin()` 实际上做了这些事情：

1. **进入事件循环**
2. **检查以下事件**：
   - 定时器是否到期
   - 是否有新消息到达
   - 是否有服务请求
   - 是否有动作目标
3. **调用对应的回调函数**
4. **重复循环**



## 3.3. 什么时候 spin 会结束？

```cpp
rclcpp::spin(node);  // 这个调用会阻塞，直到：

// 以下情况会退出 spin：
// 1. 收到 Ctrl+C 信号
// 2. 调用 rclcpp::shutdown()
// 3. 节点被显式销毁
// 4. 使用 spin_some() 或带超时的 spin
```

## 3.4. spin 的几种用法

1. 基本 spin (最常用)

   ```cpp
   rclcpp::spin(node);
   ```

2. 单线程多个 node

   单线程Spin意味着所有回调（如定时器、订阅、服务等）都在一个线程中顺序执行。

   使用方式：

   ```cpp
   rclcpp::executors::SingleThreadedExecutor executor;
   executor.add_node(node1);
   executor.add_node(node2);
   executor.spin();  // 同时处理两个节点
   ```

   适用场景：

   - **简单节点**：节点只有一个或少量回调，且回调处理时间很短。
   - **回调之间需要严格顺序**：如果回调函数必须按照发生的顺序依次处理，单线程可以保证顺序性。
   - **无阻塞操作**：回调函数中不会进行长时间的阻塞操作（如长时间计算、睡眠、I/O等待等），因为会阻塞其他回调的执行。

   优点：

   - 简单，无需担心线程安全问题。
   - 回调顺序有保障。

   缺点：

   - 如果某个回调耗时较长，会阻塞其他回调，导致实时性差。

3. 多线程

   多线程Spin使用一个线程池来处理回调，可以同时执行多个回调。

   使用方式：

   ```cpp
   rclcpp::executors::MultiThreadedExecutor executor;
   executor.add_node(node);
   executor.spin();  // 使用多个线程处理回调
   ```

   适用场景：

   - **多个回调且有的回调耗时较长**：例如，一个节点同时处理图像数据和激光雷达数据，其中图像处理较慢，而激光雷达数据需要快速响应。
   - **有阻塞操作**：如果某个回调中有I/O操作（如读写文件、网络通信）或睡眠，使用多线程可以避免阻塞其他回调。
   - **需要并行处理**：当回调之间相互独立，且可以并行处理时。

   优点：

   - 提高响应性，避免长回调阻塞其他回调。
   - 利用多核CPU，提高吞吐量。

   缺点：

   - 回调可能同时执行，需要小心数据竞争，使用锁或其他同步机制保证线程安全。
   - 回调顺序无法保证。

4. 静态单线程执行器（StaticSingleThreadedExecutor）

   这是ROS2 Galactic引入的，它使用一个静态的线程模型，适用于对性能要求较高的场景，比单线程执行器更高效。

   ```cpp
   rclcpp::executors::StaticSingleThreadedExecutor executor;
   executor.add_node(node);
   executor.spin();
   ```

5. 带超时的 spin

   ```cpp
   // 只运行10秒然后退出
   rclcpp::spin_until_future_complete(node, 
       std::chrono::seconds(10));
   ```

## 3.5. spin 选择建议

1. **默认情况下**，如果节点简单，回调函数短小且不需要并行，使用单线程。
2. **当节点有多个回调，且其中一个或多个回调可能阻塞时**，使用多线程。
3. **如果节点有多个回调，且它们之间需要共享数据**，则必须注意线程安全。如果使用多线程，需要适当的同步机制（如互斥锁）。如果使用单线程，则无需担心。


# 4. rclpy


# 5. rmw

ros rmw API: https://docs.ros.org/en/jazzy/p/rmw/


# 6. node



# 7. timer

- `create_wall_timer()`： 是一个用于创建周期性定时器的工具，但它**不依赖于 ROS 系统的仿真时间或系统时间**，而是基于“挂钟时间”。它通常用于需要稳定、真实时间间隔的任务，例如控制循环、状态监测、或与外部非ROS系统交互。


# 8. parameter

# 9. publisher

# 10. sub



# 12. rosidl


# 13. urdf


# 14. References

- ROS Index packages: https://index.ros.org/?search_packages=true
- ROS2 Packages: https://docs.ros.org/en/jazzy/
- ros doc rcl API: https://docs.ros.org/en/jazzy/p/rcl/
- rclcpp c++ API: https://docs.ros.org/en/jazzy/p/rclcpp/generated/index.html
- github ros2 rosidl: https://github.com/ros2/rosidl

