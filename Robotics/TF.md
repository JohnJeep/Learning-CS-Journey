<!--
 * @Author: JohnJeep
 * @Date: 2025-11-02 16:13:43
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-07-03 00:00:00
 * @Description: TF2 in ROS framework
 * Copyright (c) 2026 by John Jeep, All Rights Reserved. 
-->

# 1. TF2 简介

**TF（TransForm）** 是一个**坐标变换系统**，用于管理和跟踪机器人系统中所有坐标系（Frame）之间的相对位置和方向关系。它
是 ROS2 中的核心组件之一。

- **本质**：TF2 是一个**坐标变换库**。
- 作用
  - 管理机器人系统中多个坐标系（frames）之间的**动态空间关系**（如 base_link 到 camera_link 的位置和姿态）。
  - 提供 API 让节点查询任意两个坐标系在**任意时间戳**下的相对位姿（位置 + 旋转）。
  - 支持**时间缓存**、**插值**、**树状结构管理**（TF tree）。
- 使用方式
  - 通过 `tf2_ros` 包中的 `TransformBroadcaster` 发布变换。
  - 通过 `Buffer` 和 `TransformListener` 查询变换。

为什么需要 TF2？

- 激光雷达检测到障碍物在 `laser_frame` 中的位置 → 需要转换到 `map` 坐标系才能用于导航规划
- 相机识别到目标在 `camera_frame` → 转换到 `base_link` 才能控制机械臂抓取
- 多传感器融合时，数据必须统一到同一坐标系



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

## 2.1. package.xml 依赖

```xml
<depend>tf2</depend>
<depend>tf2_ros</depend>
<depend>tf2_geometry_msgs</depend>    <!-- 坐标点变换 -->
<depend>geometry_msgs</depend>
```

## 2.2. CMakeLists.txt

```cmake
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

ament_target_dependencies(my_node
  rclcpp tf2 tf2_ros tf2_geometry_msgs geometry_msgs
)
```



# 3. TF2 可视化与调试工具

`ros-humble-tf2-tools` 包提供了一系列调试和可视化 TF 变换的工具：

## 3.1. view_frames

生成当前 TF 树的 PDF 图，直观展示所有坐标系之间的父子关系：

```bash
# 生成 frames.pdf 到当前目录
ros2 run tf2_tools view_frames

# 查看生成的图
evince frames.pdf
```

## 3.2. tf2_monitor

实时监控 TF 帧之间的变换频率和时间延迟，可用于诊断变换是否正常发布：

```bash
# 监控所有 TF 帧
ros2 run tf2_ros tf2_monitor

# 只监控特定帧对
ros2 run tf2_ros tf2_monitor map base_link
```

## 3.3. tf2_echo

查看两个特定坐标系之间的实时变换数据：

```bash
# 实时打印 source_frame 相对于 target_frame 的变换
# 格式：tf2_echo <target_frame> <source_frame>
ros2 run tf2_ros tf2_echo map base_link

# 示例输出：
# At time 1234567890.123456789
# - Translation: [0.100, 0.050, 0.000]
# - Rotation: in Quaternion [0.000, 0.000, 0.707, 0.707]
```

## 3.4. 直接订阅 TF 话题

```bash
# 查看动态变换（实时，频繁更新）
ros2 topic echo /tf

# 查看静态变换（持久化，只发一次）
ros2 topic echo /tf_static

# 查看 TF 发布频率
ros2 topic hz /tf
```



# 4. 核心功能

1. **坐标变换查询**：获取任意两个坐标系间的变换
2. **时间旅行**：查询过去某时刻的坐标关系（TF Buffer 缓存历史数据）
3. **变换广播**：发布坐标系间的相对关系
4. **坐标点变换**：将点、向量、位姿从一个坐标系变换到另一个
5. **可视化**：通过 RViz2 查看坐标框架树



# 5. packages

| 包名 | 说明 |
|---|---|
| `tf2` | 核心数据结构和变换算法 |
| `tf2_ros` | ROS2 接口：Broadcaster、Listener、Buffer |
| `tf2_geometry_msgs` | 坐标点/位姿变换工具（`do_transform_*`） |
| `tf2_sensor_msgs` | 传感器消息（PointCloud2 等）坐标变换 |
| `tf2_eigen` | Eigen 矩阵格式与 TF2 互转 |
| `tf2_tools` | 调试工具（view_frames、tf2_monitor、tf2_echo） |
| `tf2_msgs` | TF2 消息定义（`TFMessage`） |

`transforms3d`：一个 Python 库，用于处理 3D 空间中的几何变换，提供的主要功能：

- 旋转矩阵、欧拉角、四元数、轴角之间的转换
- 3D 变换矩阵的操作
- 坐标系变换计算



# 6. 基本概念

## 6.1. 坐标变换（Transform）

- 描述两个坐标系之间的相对关系
- 包含**平移（Translation）** 和**旋转（Rotation）**
- 例如：机器人激光雷达相对于机器人底盘中心的位置和姿态

2 种旋转描述方式：

1. **四元数**（qx qy qz qw）：TF2 内部使用，无万向节死锁问题
2. **欧拉角**
   - roll（滚转角）：绕 X 轴旋转，rx
   - pitch（俯仰角）：绕 Y 轴旋转，ry
   - yaw（偏航角）：绕 Z 轴旋转，rz

## 6.2. TransformStamped 消息格式

TF2 使用 `geometry_msgs/msg/TransformStamped` 传递变换信息：

```
std_msgs/Header header
    builtin_interfaces/Time stamp   # 变换对应的时间戳
    string frame_id                 # 父坐标系 ID

string child_frame_id               # 子坐标系 ID

geometry_msgs/Transform transform
    geometry_msgs/Vector3 translation
        float64 x
        float64 y
        float64 z
    geometry_msgs/Quaternion rotation
        float64 x
        float64 y
        float64 z
        float64 w
```

## 6.3. 静态变换 vs 动态变换

| 类型 | 特点 | 发布方式 | 典型场景 |
|---|---|---|---|
| **静态变换** | 固定不变，发布一次 | `StaticTransformBroadcaster` | 传感器安装位置、URDF 固定关节 |
| **动态变换** | 随时间变化，持续发布 | `TransformBroadcaster` | 里程计（odom→base_link）、关节运动 |

- 静态变换发布到 `/tf_static`（QoS TRANSIENT_LOCAL，新订阅者会收到历史消息）
- 动态变换发布到 `/tf`（QoS VOLATILE，无历史）

## 6.4. 四元数与欧拉角换算

**Python 使用 `tf_transformations` 库（推荐）：**

```python
from tf_transformations import quaternion_from_euler, euler_from_quaternion

# 欧拉角（roll, pitch, yaw）→ 四元数，返回 (x, y, z, w)
q = quaternion_from_euler(0.0, 0.0, 1.57)

# 四元数 → 欧拉角
roll, pitch, yaw = euler_from_quaternion([q[0], q[1], q[2], q[3]])
```

**Python 手动计算（仅绕 Z 轴旋转的 2D 场景）：**

```python
import math

theta = 1.57  # yaw 角（弧度）
qz = math.sin(theta / 2)
qw = math.cos(theta / 2)
# 四元数为 (x=0, y=0, z=qz, w=qw)
```

**C++ 使用 `tf2::Quaternion`：**

```cpp
#include "tf2/LinearMath/Quaternion.h"

tf2::Quaternion q;
q.setRPY(0.0, 0.0, 1.57);  // roll, pitch, yaw（弧度）
// 使用 q.x(), q.y(), q.z(), q.w()
```



# 7. 时间处理

## 7.1. TF Buffer 缓存时间

`tf2_ros::Buffer` 内部缓存一段时间内的变换历史（默认 10 秒），支持查询过去某时刻的变换。

```cpp
// 设置缓存时间为 30 秒
auto tf_buffer = std::make_shared<tf2_ros::Buffer>(
    this->get_clock(),
    tf2::durationFromSec(30.0)
);
```

## 7.2. lookupTransform 时间参数

```cpp
// 查询最新可用时刻（最常用）
tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);

// 查询特定时刻（需要 Buffer 中有对应历史记录）
rclcpp::Time query_time = this->now() - rclcpp::Duration(1, 0);  // 1 秒前
tf_buffer_->lookupTransform("map", "base_link", query_time);

// 查询特定时刻，并设置等待超时
tf_buffer_->lookupTransform(
    "map", "base_link",
    query_time,
    tf2::durationFromSec(1.0)  // 最多等待 1 秒
);
```

## 7.3. 时间旅行（Time Travel）

"时间旅行"指查询过去某时刻两坐标系的相对变换，而不是当前时刻：

```cpp
// 查询 1 秒前 laser_frame 在 map 中的位姿（适合延迟处理传感器数据）
rclcpp::Time past_time = this->now() - rclcpp::Duration(1, 0);
tf_buffer_->lookupTransform("map", "laser_frame", past_time,
    tf2::durationFromSec(0.5));
```

典型应用：传感器数据有处理延迟，需要用数据采集时刻的坐标变换，而不是当前时刻。



# 8. TF 树

## 8.1. 坐标系树结构

TF2 用一棵有向树来表示所有坐标系之间的关系。树中每条边代表父子坐标系之间的变换（parent → child）。

```
map
└── odom
    └── base_link
        ├── base_footprint
        ├── laser_frame
        ├── camera_frame
        └── imu_frame
```

- 每个节点是一个坐标系（frame），用字符串 ID 标识
- 每条边是一个 `TransformStamped` 消息
- 查询任意两个坐标系的变换时，TF2 自动沿树路径组合中间变换

## 8.2. 坐标系命名惯例（REP 105）

ROS2 机器人坐标系遵循 [REP 105](https://www.ros.org/reps/rep-0105.html) 标准命名：

| 坐标系 | 说明 | 父坐标系 |
|---|---|---|
| `map` | 全局地图坐标系，静止不动，长期稳定 | 无（根节点） |
| `odom` | 里程计坐标系，连续无跳变但会漂移 | `map` |
| `base_link` | 机器人本体坐标系（底盘中心） | `odom` |
| `base_footprint` | `base_link` 在地面的投影，z=0 | `base_link` |
| `laser_frame` | 激光雷达坐标系 | `base_link` |
| `camera_frame` | 相机坐标系 | `base_link` |
| `imu_frame` | IMU 坐标系 | `base_link` |

注意：
- `map → odom`：由 SLAM 或定位模块（如 AMCL）发布，可能有跳变
- `odom → base_link`：由里程计发布，连续但累积误差



# 9. TF2 静态变换发布

静态变换（Static Transform）描述固定不变的坐标系关系，如传感器相对于机器人底盘的安装位置。

1. **静态 TF 发布器**：发布固定的坐标系关系（`StaticTransformBroadcaster`）
2. **动态 TF 发布器**：随时间移动的坐标系（`TransformBroadcaster`）

## 9.1. 命令行发布静态变换

```bash
# 格式：static_transform_publisher --x X --y Y --z Z --roll R --pitch P --yaw Y \
#        --frame-id PARENT --child-frame-id CHILD
ros2 run tf2_ros static_transform_publisher \
  --x 0.1 --y 0.0 --z 0.2 \
  --roll 0.0 --pitch 0.0 --yaw 0.0 \
  --frame-id base_link \
  --child-frame-id laser_frame
```

在 launch 文件中发布静态变换：

```python
from launch_ros.actions import Node

static_tf = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=[
        '--x', '0.1', '--y', '0.0', '--z', '0.2',
        '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
        '--frame-id', 'base_link',
        '--child-frame-id', 'laser_frame',
    ],
)
```

## 9.2. TF2 静态变换 C++ 实现

```cpp
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

class StaticFramePublisher : public rclcpp::Node
{
public:
    StaticFramePublisher() : Node("static_tf_publisher")
    {
        broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        publish_static_transform();
    }

private:
    void publish_static_transform()
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "base_link";      // 父坐标系
        t.child_frame_id = "laser_frame";     // 子坐标系

        // 平移：激光雷达安装在机器人前方 0.1m、上方 0.2m
        t.transform.translation.x = 0.1;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.2;

        // 旋转：无旋转（单位四元数）
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, 0.0);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        broadcaster_->sendTransform(t);
    }

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StaticFramePublisher>());
    rclcpp::shutdown();
    return 0;
}
```

## 9.3. TF2 静态变换 Python 实现

```python
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler

class StaticTFPublisher(Node):
    def __init__(self):
        super().__init__('static_tf_publisher')
        self.br = StaticTransformBroadcaster(self)
        self.publish_static_transform()

    def publish_static_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'laser_frame'

        t.transform.translation.x = 0.1
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.2

        q = quaternion_from_euler(0.0, 0.0, 0.0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.br.sendTransform(t)

def main():
    rclpy.init()
    node = StaticTFPublisher()
    rclpy.spin(node)
    rclpy.shutdown()
```



# 10. TF2 动态变换发布

动态变换（Dynamic Transform）描述随时间变化的坐标系关系，必须以一定频率持续发布。

## 10.1. TF2 动态变换 C++ 实现

```cpp
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

class DynamicTFPublisher : public rclcpp::Node
{
public:
    DynamicTFPublisher() : Node("dynamic_tf_publisher"), x_(0.0), theta_(0.0)
    {
        broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        // 50Hz 发布里程计坐标系变换
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&DynamicTFPublisher::publish_transform, this));
    }

private:
    void publish_transform()
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";

        t.transform.translation.x = x_;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, theta_);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        broadcaster_->sendTransform(t);

        // 模拟机器人前进
        x_ += 0.001;
    }

    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    double x_, theta_;
};
```

## 10.2. TF2 动态变换 Python 实现

```python
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class OdomTFPublisher(Node):
    def __init__(self):
        super().__init__('odom_tf_publisher')
        self.br = TransformBroadcaster(self)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        # 50Hz 发布变换
        self.timer = self.create_timer(0.02, self.publish_transform)

    def publish_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        # 仅绕 Z 轴旋转（2D 平面机器人）
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2)
        t.transform.rotation.w = math.cos(self.theta / 2)

        self.br.sendTransform(t)
```

同时广播多个变换（一次 sendTransform 调用）：

```python
# 一次性发布多个变换（减少消息数量）
transforms = []
for i, frame_id in enumerate(['arm_link1', 'arm_link2', 'arm_link3']):
    t = TransformStamped()
    t.header.stamp = self.get_clock().now().to_msg()
    t.header.frame_id = f'arm_link{i}' if i > 0 else 'base_link'
    t.child_frame_id = frame_id
    # ... 设置 translation 和 rotation
    transforms.append(t)

self.br.sendTransform(transforms)
```



# 11. TF2 坐标监听

## 11.1. 监听器初始化

`Buffer` + `TransformListener` 是查询变换的标准组合：

- `Buffer`：内部缓存变换历史，提供 `lookupTransform` 接口
- `TransformListener`：订阅 `/tf` 和 `/tf_static` 话题，将数据填充到 Buffer

**注意**：`TransformListener` 必须与节点共享生命周期，通常作为成员变量保存。

## 11.2. TF2 变换监听 C++ 实现

```cpp
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class TFListener : public rclcpp::Node
{
public:
    TFListener() : Node("tf_listener")
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TFListener::on_timer, this));
    }

private:
    void on_timer()
    {
        try {
            // lookupTransform(target_frame, source_frame, time)
            // 含义：source_frame 原点在 target_frame 中的坐标和姿态
            geometry_msgs::msg::TransformStamped tf =
                tf_buffer_->lookupTransform(
                    "map",              // target frame
                    "base_link",        // source frame
                    tf2::TimePointZero  // 最新可用时刻
                );

            RCLCPP_INFO(this->get_logger(),
                "Robot in map: x=%.2f, y=%.2f",
                tf.transform.translation.x,
                tf.transform.translation.y);

        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(),
                "Transform not available: %s", ex.what());
        }
    }

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
};
```

## 11.3. TF2 变换监听 Python 实现

```python
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.duration import Duration

class TFListenerNode(Node):
    def __init__(self):
        super().__init__('tf_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.on_timer)

    def on_timer(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                'map',              # target frame
                'base_link',        # source frame
                rclpy.time.Time(),  # 最新可用时刻
                timeout=Duration(seconds=1.0),
            )
            self.get_logger().info(
                f'Robot in map: x={tf.transform.translation.x:.2f}, '
                f'y={tf.transform.translation.y:.2f}'
            )
        except TransformException as e:
            self.get_logger().warn(f'Transform error: {e}')
```



# 12. TF2 坐标变换应用

## 12.1. 坐标点变换（Python）

将一个点从一个坐标系变换到另一个坐标系：

```python
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped
import rclpy

class PointTransformer(Node):
    def __init__(self):
        super().__init__('point_transformer')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def transform_point(self, x, y, z, from_frame, to_frame):
        point = PointStamped()
        point.header.frame_id = from_frame
        point.header.stamp = self.get_clock().now().to_msg()
        point.point.x = x
        point.point.y = y
        point.point.z = z

        try:
            tf = self.tf_buffer.lookup_transform(
                to_frame, from_frame, rclpy.time.Time()
            )
            transformed = do_transform_point(point, tf)
            return transformed.point
        except Exception as e:
            self.get_logger().warn(f'Transform failed: {e}')
            return None
```

## 12.2. 坐标点变换（C++）

```cpp
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

// 将激光雷达坐标系中的点转换到 map 坐标系
geometry_msgs::msg::PointStamped point_in_laser;
point_in_laser.header.frame_id = "laser_frame";
point_in_laser.header.stamp = this->get_clock()->now();
point_in_laser.point.x = 1.0;
point_in_laser.point.y = 0.5;
point_in_laser.point.z = 0.0;

try {
    geometry_msgs::msg::PointStamped point_in_map;
    tf_buffer_->transform(point_in_laser, point_in_map, "map");
    RCLCPP_INFO(this->get_logger(),
        "Point in map: x=%.2f, y=%.2f",
        point_in_map.point.x, point_in_map.point.y);
} catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
}
```

## 12.3. 通过 C++ 编程增加 Frame

在运行时动态添加新的坐标系（父坐标系必须已存在于 TF 树中）：

```cpp
// 在现有 base_link 下添加一个新的工具坐标系
geometry_msgs::msg::TransformStamped tool_frame;
tool_frame.header.stamp = this->get_clock()->now();
tool_frame.header.frame_id = "base_link";      // 必须已在 TF 树中
tool_frame.child_frame_id = "tool_frame";      // 新坐标系

tool_frame.transform.translation.x = 0.3;
tool_frame.transform.translation.y = 0.0;
tool_frame.transform.translation.z = 0.1;

tf2::Quaternion q;
q.setRPY(0.0, 0.0, 0.0);
tool_frame.transform.rotation = tf2::toMsg(q);

// 使用静态广播器（不变的）或动态广播器（运动的）
static_broadcaster_->sendTransform(tool_frame);
```



# 13. TF2 的 timeout 与 time travel

## 13.1. lookupTransform 的 timeout 参数

`lookupTransform` 可以设置最长等待时间。如果在 timeout 时间内变换不可用，则抛出异常：

```cpp
// C++：等待最多 1 秒直到变换可用
tf_buffer_->lookupTransform(
    "map", "base_link",
    tf2::TimePointZero,
    tf2::durationFromSec(1.0)  // timeout
);
```

```python
# Python：等待最多 1 秒
tf = self.tf_buffer.lookup_transform(
    'map', 'base_link',
    rclpy.time.Time(),
    timeout=Duration(seconds=1.0)
)
```

节点刚启动时 TF 树可能还未建立，设置适当的 timeout 可以避免立即失败。

## 13.2. C++ 编程中的应用

等待坐标系可用后再执行（常用于节点初始化阶段）：

```cpp
// 等待 map→base_link 变换可用（最多等 5 秒）
while (rclcpp::ok()) {
    try {
        tf_buffer_->lookupTransform("map", "base_link",
            tf2::TimePointZero, tf2::durationFromSec(1.0));
        RCLCPP_INFO(this->get_logger(), "TF available, starting...");
        break;
    } catch (const tf2::TransformException &) {
        RCLCPP_WARN(this->get_logger(), "Waiting for TF...");
    }
}
```

## 13.3. 常见报错与解决

| 报错 | 原因 | 解决 |
|---|---|---|
| `"map" does not exist` | 该坐标系尚未被任何节点发布 | 确认发布节点已启动 |
| `Lookup would require extrapolation into the past` | 查询时间戳早于 Buffer 最早记录 | 增大 Buffer 缓冲时间，或用 `TimePointZero` |
| `Lookup would require extrapolation into the future` | 查询时间戳超过最新 TF 记录 | 检查时钟同步，或用 `TimePointZero` |
| `Could not find a connection between ... and ...` | 坐标系树中两节点不连通 | 运行 `view_frames` 检查树结构 |



# 14. References

- [TF2 官方教程](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Tf2-Main.html)
- [REP 105: 坐标系命名约定](https://www.ros.org/reps/rep-0105.html)
- [tf2_geometry_msgs API](https://docs.ros2.org/latest/api/tf2_geometry_msgs/)
- [tf2_ros API（C++）](https://docs.ros2.org/latest/api/tf2_ros/)
- [动手学 ROS2 - TF2](http://fishros.com/d2lros2/)
