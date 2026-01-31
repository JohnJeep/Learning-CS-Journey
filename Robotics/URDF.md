<!--
 * @Author: JohnJeep
 * @Date: 2025-11-02 16:01:47
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-01-31 16:55:43
 * @Description: URDF
 * Copyright (c) 2025 by John Jeep, All Rights Reserved. 
-->

- [1. Introduction to URDF](#1-introduction-to-urdf)
- [2. 标签](#2-标签)
  - [2.1. `<link>`](#21-link)
  - [2.2. `<joint>`](#22-joint)
  - [2.3. `link` 与 `join`t 的关系是什么？](#23-link-与-joint-的关系是什么)
    - [2.3.1. **树状结构：父子关系**](#231-树状结构父子关系)
  - [2.4. `<transmission>`](#24-transmission)
    - [2.4.1. 为什么需要 `<transmission>`？](#241-为什么需要-transmission)
    - [2.4.2. 基本语法（ROS 2 / `ros2_control` 兼容格式）](#242-基本语法ros-2--ros2_control-兼容格式)
    - [2.4.3. 关键子元素说明](#243-关键子元素说明)
    - [2.4.4. ROS 2 中的现代替代方案：`<ros2_control>`](#244-ros-2-中的现代替代方案ros2_control)
      - [2.4.4.1. 示例（ROS 2 推荐方式）：](#2441-示例ros-2-推荐方式)
    - [2.4.5. 何时使用 `<transmission>`？](#245-何时使用-transmission)
    - [2.4.6. 对比](#246-对比)
- [3. 注意点](#3-注意点)
- [4. Xarco](#4-xarco)
- [5. packages](#5-packages)
  - [5.1. ros-humble-urdf-tutorial](#51-ros-humble-urdf-tutorial)
    - [5.1.1. 安装](#511-安装)
    - [5.1.2. 启动示例模型](#512-启动示例模型)
  - [5.2. check\_urdf](#52-check_urdf)
- [6. References](#6-references)


## 1. Introduction to URDF

URDF(Unified Robot Description Format)，全称是**统一机器人描述格式**。URDF 是一种 XML 格式，用于描述机器人模型的结构、关节、连杆等信息，包括底盘、摄像头、激光雷达、机械臂和不同关节的自由度。

URDF 文件主要用于 ROS（Robot Operating System）中，帮助机器人软件理解机器人的物理结构和运动学特性，从而实现仿真、运动规划和控制等功能。

机器人一般是由**硬件结构、驱动系统、传感器系统、控制系统**四大部分组成。
- 硬件结构就是底盘、外壳、电机等实打实可以看到的设备；
- 驱动系统就是可以驱使这些设备正常使用的装置，比如电机的驱动器，电源管理系统等；
- 传感系统包括电机上的编码器、板载的IMU、安装的摄像头、雷达等等，便于机器人感知自己的状态和外部的环境；
- 控制系统就是我们开发过程的主要载体了，一般是树莓派、电脑等计算平台，以及里边的操作系统和应用软件。

## 2. 标签

### 2.1. `<link>`

表示机器人中的一个刚性连杆（rigid body），例如：

- 底盘（base_link）
- 轮子（wheel_link）
- 机械臂的某一段（arm_link_1）

每个 `link` 可以包含：

- **视觉属性**（`<visual>`）：用于 RViz 显示
- **碰撞属性**（`<collision>`）：用于 Gazebo 或运动规划中的碰撞检测
- **惯性属性**（`<inertial>`）：用于物理仿真（如 Gazebo）

```xml
<link name="base_link">
  <visual>
    <geometry><box size="0.5 0.5 0.1"/></geometry>
  </visual>
</link>
```

详细标签内容：

| 标签名      | 描述                                                         |
| ----------- | ------------------------------------------------------------ |
| <link>      | 连接可视化、碰撞、惯性的属性                                 |
| <visual>    | 可视化属性，物体形状                                         |
| <collision> | 碰撞计算属性                                                 |
| <inertial>  | 惯性属性                                                     |
| <geometry>  | 输入模型的形状。对物体形状的描述，如比 圆柱体、立方体、球、网格 |
| <rigin>     | 相对坐标的移动和旋转                                         |
| <material>  | 物体的材质，比如颜色、纹理                                   |
| <mass>      | 质量属性                                                     |

### 2.2. `<joint>`

- 定义**两个 link 之间的连接方式和运动自由度**。
- 必须指定：
  - `name`：关节名称
  - `type`：关节类型（见下文）
  - `parent`：父 link
  - `child`：子 link
  - `origin`：子 link 相对于父 link 的位姿（位置 + 旋转）

```xml
<joint name="wheel_to_base" type="continuous">
  <parent link="base_link"/>
  <child link="wheel_link"/>
  <origin xyz="0 0.3 0" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
</joint>
```

详细标签内容：

| 标签     | 描述                                                         |
| -------- | ------------------------------------------------------------ |
| <joint>  | 关节类型的设置                                               |
| <parent> | 关节的父连接                                                 |
| <child>  | 关节的子连接                                                 |
| <origin> | 父连接坐标系转换为子连接坐标系                               |
| <axis>   | 设置旋转轴                                                   |
| <limit>  | 设置关节的运动限制和力矩限制（仅当关节是 revolute 或 prismatic） |

**type（joint 类型）**

joint 的类型决定运动方式。

| 类型         | 说明                                                         | 自由度         |
| ------------ | ------------------------------------------------------------ | -------------- |
| `fixed`      | 固定连接，无相对运动                                         | 0              |
| `revolute`   | 旋转关节（有角度限制）                                       | 1（旋转）      |
| `continuous` | 连续旋转关节（如轮子，无角度限制）                           | 1（旋转）      |
| `prismatic`  | 滑动关节（直线移动）。沿某一轴移动的关节，带有位置极限。     | 1（平移）      |
| `floating`   | 完全自由（6 DOF）。允许进行平移、旋转运动。                  | 6              |
| `planar`     | 平面内自由移动。允许在平面正交方向（xzyz，rx ry rz）方向上移动或旋转。 | 3（x, y, yaw） |

> 大多数轮式机器人使用 `continuous` 关节；机械臂常用 `revolute`。



### 2.3. `link` 与 `join`t 的关系是什么？

在 URDF（Unified Robot Description Format）中，**`link`** 和 **`joint`**（是两个核心元素，它们共同定义了机器人的**结构**和**运动关系**。**`link` 是“刚体部件”，`joint` 是“连接这些部件的关节”。**

#### 2.3.1. **树状结构：父子关系**

- URDF 描述的是一个**树形结构**（不能有闭环！）。
- 必须有一个**根 link**（root link），它没有父节点。
- 每个 `joint` 连接一个 **parent link** 和一个 **child link**。
- 通过 `joint`，你可以从根 link 出发，构建整个机器人 kinematic tree（运动学树）。

```text
base_link
   ├── front_left_wheel (via joint "fl_wheel_joint")
   ├── front_right_wheel (via joint "fr_wheel_joint")
   └── arm_base
        └── arm_link_1 (via joint "shoulder")
             └── arm_link_2 (via joint "elbow")
```

**关键关系总结**

| 方面     | `link`                   | `joint`                                    |
| -------- | ------------------------ | ------------------------------------------ |
| 角色     | “身体部件”               | “连接件 / 关节”                            |
| 是否可动 | 本身是刚体，不可变形     | 定义 link 之间的相对运动                   |
| 数量关系 | N 个 link                | 最多 N−1 个 joint（树结构）                |
| 必需性   | 至少 1 个（根 link）     | 如果只有 1 个 link，则不需要 joint         |
| 坐标系   | 每个 link 有自己的坐标系 | 定义 child link 坐标系相对于 parent 的位姿 |

### 2.4. `<transmission>`

在 URDF（Unified Robot Description Format）中，`<transmission>` 标签 **不是 URDF 原生标准的一部分**，而是为了与 **ROS 控制系统（如 `ros_control` 或 ROS 2 中的 `ros2_control`）集成**而引入的扩展。它的主要作用是：

> **描述执行器（如电机）如何将力/扭矩传递到关节（joint），从而实现对机器人的控制。**

------

#### 2.4.1. 为什么需要 `<transmission>`？

URDF 本身只描述**几何结构和运动学关系**（link + joint），但不包含：

- 哪个电机驱动哪个关节？
- 传动比是多少？
- 使用什么类型的执行器（位置/速度/力控制）？

而这些信息对于**仿真（如 Gazebo）** 和 **真实机器人控制（通过 `ros2_control`）** 至关重要。`<transmission>` 就是用来填补这一空白的。

------

#### 2.4.2. 基本语法（ROS 2 / `ros2_control` 兼容格式）

在 ROS 2 Humble 及以后版本中，推荐使用 **`ros2_control` 的新 URDF 标签体系**，但 `<transmission>` 仍被部分旧系统（或 Gazebo Classic）使用。以下是典型结构：

```xml
<transmission name="wheel_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="left_wheel_joint">
    <hardwareInterface>PositionJointInterface</hardwareInterface>
  </joint>
  <actuator name="left_motor">
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

> ⚠️ 注意：在 **ROS 2 的现代实践中（尤其是使用 `ros2_control`）**，官方更推荐直接在 URDF 中使用 `<ros2_control>` 标签（见下文对比），而不是 `<transmission>`。

------

#### 2.4.3. 关键子元素说明

| 元素                    | 说明                                                         |
| ----------------------- | ------------------------------------------------------------ |
| `<type>`                | 传动类型，常见： <br />• `SimpleTransmission`（最常用）<br /> • `DifferentialTransmission`（差速） <br />• `FourBarLinkageTransmission` |
| `<joint>`               | 指定该传动作用于哪个关节（必须与 URDF 中的 joint 名称一致）  |
| `<hardwareInterface>`   | 指定控制接口类型：<br /> • `PositionJointInterface` <br />• `VelocityJointInterface` <br />• `EffortJointInterface`（力/扭矩） |
| `<actuator>`            | 描述执行器（电机）                                           |
| `<mechanicalReduction>` | 舵机与关节之间的齿轮比（例如齿轮减速比）。值 >1 表示减速，<1 表示增速。 |

#### 2.4.4. ROS 2 中的现代替代方案：`<ros2_control>`

从 ROS 2 Foxy/Humble 开始，**官方推荐使用 `<ros2_control>` 标签**直接在 URDF 中定义硬件接口和控制器，**不再依赖 `<transmission>`**。

##### 2.4.4.1. 示例（ROS 2 推荐方式）：

```xml
<ros2_control name="GazeboSystem" type="system">
  <hardware>
    <plugin>gazebo_ros2_control/GazeboSystem</plugin>
  </hardware>
  <joint name="left_wheel_joint">
    <command_interface name="velocity">
      <min>-100.0</min>
      <max>100.0</max>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

这种方式更清晰、模块化，并与 `controller_manager` 紧密集成。

------

#### 2.4.5. 何时使用 `<transmission>`？

| 场景                                                   | 是否使用 `<transmission>`                  |
| ------------------------------------------------------ | ------------------------------------------ |
| **Gazebo Classic（ROS 1 或 ROS 2 + gazebo_ros_pkgs）** | ✅ 需要，用于加载 `gazebo_ros_control` 插件 |
| **Ignition Gazebo / Gazebo Harmonic（ROS 2）**         | ❌ 不再使用，改用 `<ros2_control>`          |
| **真实机器人 + ros2_control**                          | ❌ 推荐直接用 `<ros2_control>` 标签         |
| **仅 RViz 可视化（无控制/仿真）**                      | ❌ 完全不需要                               |

------

#### 2.4.6. 对比

| 对比项            | `<transmission>`            | `<ros2_control>`（现代 ROS 2） |
| ----------------- | --------------------------- | ------------------------------ |
| 所属体系          | `ros_control`（ROS 1 遗留） | `ros2_control`（ROS 2 原生）   |
| 用途              | 描述执行器→关节的传动关系   | 直接定义硬件接口和控制命令     |
| 是否推荐（ROS 2） | 仅限 Gazebo Classic         | ✅ 强烈推荐                     |
| 可读性            | 较低                        | 高，结构清晰                   |

> ✅ **建议**：如果你使用的是 **ROS 2 Humble + Gazebo（经典版）**，可能仍需 `<transmission>`；
> 如果使用 **ROS 2 + Ignition/Gazebo Sim 或真实机器人**，请优先学习 `<ros2_control>` 标签。



## 3. 注意点

1. **闭环结构**：URDF 不支持闭环（如四连杆机构），需用 SDF（Gazebo 格式）或 ROS 2 的 `<ros2_control>` + 插件处理。
2. **缺少根 link**：必须有一个不作为任何 joint 的 child 的 link。
3. **joint 的 parent/child 指向不存在的 link**：会导致 `robot_state_publisher` 报错。



## 4. Xarco

Xacro（XML Macros）是 URDF 的预处理器，是 URDF 的升级版，支持：

- 宏定义（`<xacro:macro>`）
- 常量（`<xacro:property>`）
- 数学表达式（`${...}`）
- 条件语句（`<xacro:if>`）

- 最终仍需转换为标准 URDF 才能被 `robot_state_publisher` 使用。
- 在 ROS 2 中，`xacro` 是一个独立的可执行程序，通常通过 launch 文件自动调用。



## 5. packages

### 5.1. ros-humble-urdf-tutorial 

`ros-humble-urdf-tutorial` 是 ROS 2 Humble 版本中用于学习 URDF（Unified Robot Description Format）的一个官方教学包。

#### 5.1.1. 安装

```bash
sudo apt update
sudo apt install ros-humble-urdf-tutorial
```

#### 5.1.2. 启动示例模型

该包提供了多个示例 URDF 文件和 launch 文件，用于在 RViz 中可视化机器人模型。文件位于 `/opt/ros/humble/share/urdf_tutorial/urdf/` 目录下。

**文件说明**

- **01-myfirst.urdf**：最简单的单连杆模型。
- **02-multipleshapes.urdf**：多个几何形状（box, cylinder, sphere）。
- **03-origins.urdf**：设置 `<origin>` 定义坐标偏移。
- **04-materials.urdf**：添加颜色材质。
- **05-visual.urdf**：区分 visual（可视化）与 collision（碰撞）属性。
- **06-flexible.urdf**：引入关节（joint），构建多连杆结构。
- **07-physics.urdf**：添加惯性（inertial）和物理属性。
- **08-macroed.urdf.xacro**：使用 Xacro 宏简化重复结构。

```bash
# 启动一个基本的模型
ros2 launch urdf_tutorial display.launch.py model:=urdf/01-myfirst.urdf
```

### 5.2. check_urdf

`check_urdf` 是 URDF 语法检查是否正确的工具。

```bash
sudo apt install liburdfdom-tools
check_urdf your_model.urdf
```


## 6. References

- 官方教程文档（ROS 2）：
  [https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Building-a-Visual-Robot-Model-with-URDF-from-Scratch.html](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Building-a-Visual-Robot-Model-with-URDF-from-Scratch.html?spm=5176.28103460.0.0.2d3c6308fzDXKK)
- URDF 官方规范：
  [http://wiki.ros.org/urdf/XML](http://wiki.ros.org/urdf/XML?spm=5176.28103460.0.0.2d3c6308fzDXKK)
- Xacro 教程（用于简化 URDF）：
  [https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Using-Xacro-to-Clean-Up-a-URDF-File.html](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Using-Xacro-to-Clean-Up-a-URDF-File.html?spm=5176.28103460.0.0.2d3c6308fzDXKK)
- humble Tutorials: 
  https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html

