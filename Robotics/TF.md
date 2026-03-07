<!--
 * @Author: JohnJeep
 * @Date: 2025-11-02 16:13:43
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-03-07 15:18:31
 * @Description: TF2 in ROS framework
 * Copyright (c) 2026 by John Jeep, All Rights Reserved. 
-->

- [1. TF2 简介](#1-tf2-简介)
- [2. 安装与运行](#2-安装与运行)
- [3. TF2 可视化与调试工具介绍](#3-tf2-可视化与调试工具介绍)
- [4. 核心功能](#4-核心功能)
- [5. packages](#5-packages)
- [6. 基本概念](#6-基本概念)
    - [6.0.1. 坐标变换（Transform）](#601-坐标变换transform)
    - [6.0.2. 坐标系树（TF Tree）](#602-坐标系树tf-tree)
    - [6.0.3. 时间戳（Timestamp）](#603-时间戳timestamp)
- [7. 时间处理](#7-时间处理)
- [8. TF 树](#8-tf-树)
- [9. TF2 静态变换发布](#9-tf2-静态变换发布)
  - [9.1. TF2 静态变换 C++ 实现](#91-tf2-静态变换-c-实现)
- [10. TF2 动态变换发布](#10-tf2-动态变换发布)
  - [10.1. TF2 动态变换 C++ 实现](#101-tf2-动态变换-c-实现)
- [11. TF2 坐标监听](#11-tf2-坐标监听)
  - [11.1. TF2 变换监听 C++ 实现](#111-tf2-变换监听-c-实现)
- [12. TF2 坐标系](#12-tf2-坐标系)
  - [12.1. 通过 C++ 编程增加 Frame](#121-通过-c-编程增加-frame)
- [13. TF2 的 timeout 与 time travel 介绍](#13-tf2-的-timeout-与-time-travel-介绍)
  - [13.1. C++ 编程中的应用](#131-c-编程中的应用)
- [14. References](#14-references)

# 1. TF2 简介

**TF（TransForm）** 是一个**坐标变换系统**，用于管理和跟踪机器人系统中所有坐标系（Frame）之间的相对位置和方向关系。它是ROS（机器人操作系统）中的核心组件之一。

- **本质**：TF2 是一个**坐标变换库**。
- 作用
  - 管理机器人系统中多个坐标系（frames）之间的**动态空间关系**（如 base_link 到 camera_link 的位置和姿态）。
  - 提供 API 让节点查询任意两个坐标系在**任意时间戳**下的相对位姿（位置 + 旋转）。
  - 支持**时间缓存**、**插值**、**树状结构管理**（TF tree）。
- 使用方式
  - 通过 `tf2_ros` 包中的 `TransformBroadcaster` 发布变换。
  - 通过 `Buffer` 和 `TransformListener` 查询变换。

# 2. 安装与运行

查看系统中是否安装 `tf2`

```bash
jacky:~/ $ ros2 pkg list | grep tf2                                                         
tf2
tf2_bullet
tf2_eigen
tf2_eigen_kdl
tf2_geometry_msgs
tf2_kdl
tf2_msgs
tf2_py
tf2_ros
tf2_ros_py
tf2_sensor_msgs
tf2_tools
```

查看核心包版本

```bash
ros2 pkg xml tf2 --tag version
ros2 pkg xml tf2_ros --tag version
```



# 3. TF2 可视化与调试工具介绍



# 4. 核心功能

1. **坐标变换查询**：获取任意两个坐标系间的变换
2. **时间旅行**：查询过去或预测未来的坐标关系
3. **变换广播**：发布坐标系间的相对关系
4. **可视化**：通过rviz等工具查看坐标框架



# 5. packages

ros-humble-tf2-tools：安装TF2相关的工具集，提供了一系列调试和可视化TF变换的工具；包含的主要工具有：

- view_frames：生成TF帧关系的PDF图；
- tf2_monitor：监控TF帧之间的变换关系；
- tf2_echo：查看两个特定帧之间的变换数据；

transforms3d：一个Python库，用于处理3D空间中的几何变换，提供的主要功能：

- 旋转矩阵、欧拉角、四元数、轴角之间的转换；
- 3D变换矩阵的操作；
- 坐标系变换计算。



# 6. 基本概念

### 6.0.1. 坐标变换（Transform）

- 描述两个坐标系之间的相对关系
- 包含**平移（Translation）** 和**旋转（Rotation）**
- 例如：机器人手爪相对于机器人底座的位置和姿态



2 种姿态描述

1. 四元数形式（qx qy qz qw）
2. 欧拉角形式

   - yaw: 偏航角 rz

   - pitch: 俯仰角 ry

   - roll: 滚转角 rx




### 6.0.2. 坐标系树（TF Tree）

- 所有坐标系通过父子关系连接成树状结构
- 每个变换都有：
  - **父坐标系（Parent Frame）**
  - **子坐标系（Child Frame）**
  - **变换关系（Transform）**

### 6.0.3. 时间戳（Timestamp）

- TF 记录变换的时间，支持查询历史变换
- 可以回答“10秒前摄像头相对于底座的位置”



# 7. 时间处理



# 8. TF 树



# 9. TF2 静态变换发布

1. **静态TF发布器**：发布固定的坐标系关系
2. **动态TF发布器**：随时间移动的坐标系

## 9.1. TF2 静态变换 C++ 实现



# 10. TF2 动态变换发布



## 10.1. TF2 动态变换 C++ 实现



# 11. TF2 坐标监听



## 11.1. TF2 变换监听 C++ 实现



# 12. TF2 坐标系



##  12.1. 通过 C++ 编程增加 Frame





# 13. TF2 的 timeout 与 time travel 介绍



## 13.1. C++ 编程中的应用


# 14. References
