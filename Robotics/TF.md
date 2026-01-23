# Introduction

**TF（TransForm）** 是一个**坐标变换系统**，用于管理和跟踪机器人系统中所有坐标系（Frame）之间的相对位置和方向关系。它是ROS（机器人操作系统）中的核心组件之一。

- **本质**：TF2 是一个**坐标变换库**。
- 作用
  - 管理机器人系统中多个坐标系（frames）之间的**动态空间关系**（如 base_link 到 camera_link 的位置和姿态）。
  - 提供 API 让节点查询任意两个坐标系在**任意时间戳**下的相对位姿（位置 + 旋转）。
  - 支持**时间缓存**、**插值**、**树状结构管理**（TF tree）。
- 使用方式
  - 通过 `tf2_ros` 包中的 `TransformBroadcaster` 发布变换。
  - 通过 `Buffer` 和 `TransformListener` 查询变换。

# 核心功能

1. **坐标变换查询**：获取任意两个坐标系间的变换
2. **时间旅行**：查询过去或预测未来的坐标关系
3. **变换广播**：发布坐标系间的相对关系
4. **可视化**：通过rviz等工具查看坐标框架

# install

```bash
# 查看系统中是否安装 tf2
ubt:~/ $ ros2 pkg list | grep tf2                                                         
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



```bash
# 查看核心包版本
ros2 pkg xml tf2 --tag version
ros2 pkg xml tf2_ros --tag version

```

# packages

ros-humble-tf2-tools：安装TF2相关的工具集，提供了一系列调试和可视化TF变换的工具；包含的主要工具有：

- view_frames：生成TF帧关系的PDF图；
- tf2_monitor：监控TF帧之间的变换关系；
- tf2_echo：查看两个特定帧之间的变换数据；

transforms3d：一个Python库，用于处理3D空间中的几何变换，提供的主要功能：

- 旋转矩阵、欧拉角、四元数、轴角之间的转换；
- 3D变换矩阵的操作；
- 坐标系变换计算。



# 基本概念

### 1. 坐标变换（Transform）

- 描述两个坐标系之间的相对关系
- 包含**平移（Translation）** 和**旋转（Rotation）**
- 例如：机器人手爪相对于机器人底座的位置和姿态



**2 种姿态描述vscode-terminal:/55f716cdde799fee830490c286d5e6e9/2**

1. 四元数形式（qx qy qz qw）
2. 欧拉角形式

   - yaw: 偏航角 rz

   - pitch: 俯仰角 ry

   - roll: 滚转角 rx




### 2. 坐标系树（TF Tree）

- 所有坐标系通过父子关系连接成树状结构
- 每个变换都有：
  - **父坐标系（Parent Frame）**
  - **子坐标系（Child Frame）**
  - **变换关系（Transform）**

### 3. 时间戳（Timestamp）

- TF 记录变换的时间，支持查询历史变换
- 可以回答“10秒前摄像头相对于底座的位置”





# 广播与监听



static_transform_publisher



# TF 树



# 时间处理





# 静态 动态TF





1. **静态TF发布器**：发布固定的坐标系关系

2. **动态TF发布器**：随时间移动的坐标系

   
