<!--
 * @Author: JohnJeep
 * @Date: 2026-11-02 10:38:59
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-11-02 17:55:30
 * @Description: ROS2 Internal Core Modules
 * Copyright (c) 2026 by John Jeep, All Rights Reserved.
-->

# ROS2 Internal

核心模块

| 模块(仓库)                      | 模块名称                         | 主要功能                                                                                                                          |
| :------------------------------ | :------------------------------- | :-------------------------------------------------------------------------------------------------------------------------------- |
| `ros2/ros2`                     | 元仓库                           | 汇总 ROS 2 工作空间配置和子仓库依赖清单（`.repos` 文件）。                                                                        |
| `ros2/rcl`                      | ROS Client Library (C语言实现层) | ROS 2 客户端库的核心 C 语言实现，向下封装中间件接口，向上提供基础通信功能。                                                       |
| `ros2/rclcpp`                   | ROS Client Library (C++ API)     | ROS 2 的 C++ 客户端库，基于 rcl 实现，为 C++ 开发者提供节点、发布者等核心 API。                                                   |
| `ros2/rclpy`                    | ROS Client Library (Python API)  | ROS 2 的 Python 客户端库，基于 rcl 实现，为 Python 开发者提供 ROS 2 编程接口。                                                    |
| `ros2/rmw`                      | ROS 中间件接口                   | ROS 2 通信模块的抽象接口层，定义了与底层 DDS（数据分发服务）交互的标准 API。                                                      |
| `ros2/rmw_implementation`       | ROS 中间件实现抽象层             | 负责在运行时动态加载并绑定具体的 DDS 中间件实现（如 Fast DDS、Cyclone DDS）。                                                     |
| `eProsima/Fast-DDS`             | Fast-DDS实现库                   | DDS 实现层：由 eProsima 开发的高性能 DDS 中间件，是 ROS 2 默认使用的底层通讯框架。                                                |
| `ros2/rmw_fastrtps`             | Fast-DDS 中间件封装库            | 将 eProsima Fast DDS 适配为 ROS 2 中间件接口（RMW）的具体实现。                                                                   |
| `eclipse-cyclonedds/cyclonedds` | CycloneDDS实现库                 | DDS 实现层：由 Eclipse 基金会开发并维护的高性能、轻量级 DDS 通信框架。                                                            |
| `ros2/rmw_cyclonedds`           | CycloneDDS 中间件封装库          | 将 Eclipse Cyclone DDS 适配为 ROS 2 中间件接口（RMW）的具体实现。                                                                 |
| `ros2/rmw_connextdds`           | ConnextDDS 中间件封装库          | 将 RTI Connext DDS 适配为 ROS 2 中间件接口（RMW）的具体实现。                                                                     |
| `ros2/rmw_dds_common`           | DDS 通用共享库                   | 提供各个 DDS 中间件实现共享的通用代码（如节点图缓存等工具）。                                                                     |
| `ros2/rosidl`                   | ROS接口定义库                    | 提供 ROS 2 接口定义语言（IDL）的代码生成工具，将 `.msg`、`.srv`、`.action` 等文件转换为 C++、Python 等多语言代码及 DDS 兼容文件。 |
| `ros2/rosidl_core`              | 接口定义核心元仓库               | 聚合 rosidl 相关的核心子包（如生成器、类型支持等）的元仓库。                                                                      |
| `ros2/rcl_interfaces`           | rcl 接口定义                     | 定义 rcl 库使用的底层消息和服务接口（如参数、日志级别等）。                                                                       |
| `ros2/common_interfaces`        | 通用标准消息接口                 | 汇总 ROS 2 开发中最常用的标准消息包（如 `std_msgs`, `sensor_msgs`, `geometry_msgs` 等）。                                         |
| `ros2/rcl_logging`              | 日志记录抽象接口                 | 定义日志后端需实现的 C API 抽象层，供 rcl 统一管理日志输出。                                                                      |
| `ros2/ros2cli`                  | 命令行工具                       | 提供常用的 ROS 2 终端命令行工具（如 `ros2 node`, `ros2 topic`, `ros2 service` 等）的实现。                                        |
| `ros2/ament_cmake_ros`          | 构建系统扩展包                   | 为 ROS 2 项目提供 `ament_cmake` 的扩展功能和构建辅助脚本。                                                                        |
| `ros2/urdf`                     | 机器人模型描述工具库             | 提供 URDF（统一机器人描述格式）文件的解析器、C++ 库及验证工具。                                                                   |
| `ros2/design`                   | 架构设计文档                     | ROS 2 的设计文档、架构说明和各项特性的提案（REP）仓库。                                                                           |
| `eclipse-iceoryx/iceoryx`       | 共享内存通信中间件               | 面向高性能进程间通信（IPC）的共享内存中间件，支持零拷贝数据传输。                                                                 |

## ROS2 Action





# References

- https://github.com/ros2/rclcpp.git：ROS 2 的 C++ 客户端库，提供节点、发布者、订阅者等核心 API。

- https://github.com/ros2/rosidl.git：ROS 2 接口定义语言（IDL）工具链，用于生成消息、服务等代码。
- https://github.com/ros2/rmw_implementation.git：RMW 中间件实现的抽象层，负责在运行时加载具体 DDS 实现。
- https://github.com/ros2/rmw_fastrtps.git：基于 eProsima Fast DDS 的 ROS 2 中间件实现。
- https://github.com/ros2/rmw.git：ROS 中间件抽象接口（RMW），定义与 DDS 实现交互的 C API。
- https://github.com/ros2/rmw_connextdds.git：基于 RTI Connext DDS 的 ROS 2 中间件实现。
- https://github.com/ros2/common_interfaces.git：ROS 2 常用标准消息接口集合（如 std_msgs、sensor_msgs 等）。
- https://github.com/ros2/rclc.git：ROS 2 的 C 客户端库，封装 RMW 并提供节点、参数等通用功能。
- https://github.com/ros2/rcl.git: ROS 2 通用客户端核心层。
- https://github.com/ros2/rcl_interfaces.git：rcl 库使用的接口定义（如参数、日志等消息与服务）。
- https://github.com/ros2/ament_cmake_ros.git：为 ROS 2 项目提供的 ament_cmake 扩展与构建辅助功能。
- https://github.com/ros2/rmw_dds_common.git：多个 DDS 中间件实现共享的通用代码与工具。
- https://github.com/ros2/rosidl_core.git：rosidl 核心包的元仓库，聚合消息生成相关子包。
- https://github.com/ros2/urdf.git：URDF（统一机器人描述格式）的解析与工具库。
- https://github.com/ros2/design.git：ROS 2 的设计文档与架构说明仓库。
- https://github.com/eclipse-iceoryx/iceoryx.git：面向高性能进程间通信的共享内存中间件（零拷贝 IPC）。
- https://github.com/ros2/rcl_logging.git：ROS 2 的日志记录抽象接口，定义日志后端需实现的 C API，供 rcl 统一调用。
- https://github.com/ros2/rmw_cyclonedds.git：基于 Eclipse Cyclone DDS 的 ROS 2 中间件实现。

