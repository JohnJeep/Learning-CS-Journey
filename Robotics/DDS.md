<!--
 * @Author: JohnJeep
 * @Date: 2025-10-26 16:46:49
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-11-13 20:34:39
 * @Description: 
 * Copyright (c) 2025 by John Jeep, All Rights Reserved. 
-->

## DDS

DDS的全称是**Data Distribution Service**，也就是**数据分发服务**，2004年由**对象管理组织OMG**发布和维护，是一套专门为**实时系统**设计的**数据分发/订阅标准**，最早应用于美国海军， 解决舰船复杂网络环境中大量软件升级的兼容性问题，现在已经成为强制标准。

DDS强调**以数据为中心**，可以提供丰富的**服务质量策略**(QoS)，以保障数据进行实时、高效、灵活地分发，可满足各种分布式实时通信应用需求

DDS 是 ROS2 的 底层通信机制，负责节点间的数据传输、发现和通信管理。它是一种通信标准和一套API 定义。ROS2 采用 eProsima FastDDS 为默认的 DDS。

### 一、DDS 的核心原理

DDS 是一个面向实时、分布式系统的中间件标准（由 OMG 组织定义），核心目标是高效、可靠地实现动态节点间的数据共享。其核心原理可总结为以下几点：

#### 1. 以 “数据为中心” 的通信模型

DDS 与 ROS 1 的 “以节点为中心” 不同，它将**数据本身**作为通信的核心，节点通过 “发布 / 订阅” 数据主题（Topic）进行交互，无需关心数据的发送方或接收方是谁。

- 节点只需声明自己关心的数据（订阅）或产生的数据（发布），DDS 会自动处理数据的路由和分发。
- 这种模型更适合动态系统（如节点随时加入 / 退出），无需预先配置通信拓扑。

一句话概括：**DDS 是一个以 数据 为中心的 发布订阅模型。**

#### 2. 关键概念：Domain、Topic、QoS

- **Domain（域）**：DDS 通过 “域” 隔离不同的通信群体，只有处于同一域（Domain ID 相同）的节点才能相互发现和通信。例如，ROS 2 默认使用`Domain ID=0`，若需隔离多个系统，可修改该值（如`export ROS_DOMAIN_ID=1`）。

- **Topic（主题）**：数据的逻辑标识，由 “名称” 和 “数据类型” 共同定义（两者缺一不可）。例如，ROS 2 中的`/chatter`主题，数据类型为`std_msgs/msg/String`，只有发布和订阅相同名称 + 类型的节点才能通信。

- **QoS（服务质量）**

  在 ROS 2 中，QoS（Quality of Service，服务质量）策略是控制节点间数据传输行为的核心机制，直接影响通信的可靠性、实时性、数据持久性等。配置和使用 QoS 需要结合具体场景（如传感器数据、控制指令、日志信息等），确保数据传输符合需求。

  QoS是DDS的灵魂，也是你作为开发者必须掌握的工具。它允许你精确控制通信行为。在ROS 2中，你可以在创建 `Publisher` 或 `Subscriber` 时指定QoS。

  **常见且重要的QoS策略：**

  | QoS策略                 | 功能描述                                                 | 常用场景                                                     |
  | :---------------------- | :------------------------------------------------------- | :----------------------------------------------------------- |
  | **Reliability**(可靠性) | 确保数据可靠交付。                                       |                                                              |
  | `RELIABLE`              | 保证数据按顺序送达，丢失会重传。                         | **命令控制**（如 `/cmd_vel`），**关键状态**。                |
  | `BEST_EFFORT`           | 尽最大努力交付，可能丢失数据，但延迟更低。               | **高频传感器数据**（如激光雷达、摄像头），丢失一帧无所谓。   |
  | **Durability**(持久性)  | 控制历史数据的持久化。                                   |                                                              |
  | `VOLATILE`              | 不保存历史数据。新加入的订阅者收不到之前的数据。         | 大多数传感器数据。                                           |
  | `TRANSIENT_LOCAL`       | 发布者为最后N个样本保留历史。新订阅者能收到最近的数据。  | **地图**、**参数配置**、**静态变换**。新节点启动后能立刻获取到地图。 |
  | **History**(历史记录)   | 当处理速度跟不上发布速度时，如何管理缓存中的数据。       |                                                              |
  | `KEEP_LAST`             | 只保留最新的N个样本。                                    | 最常见的选择。需要指定 `depth`。                             |
  | `KEEP_ALL`              | 保留所有样本，直到被订阅者取走。                         | 与 `RELIABLE` 配合，确保不丢任何数据。                       |
  | **Depth**               | 与 `KEEP_LAST` 配合，定义历史队列的大小。                | 根据系统处理能力和数据频率设定。例如 `depth=10`。            |
  | **Deadline**(截止时间)  | 指定数据更新的最大间隔，超过则触发回调（确保数据时效性） | 用于**时序监控**。如果发布者未在指定周期内发布数据，会触发回调。 |
  | **Lifespan**(生命周期)  | 数据在发布后的“有效期”，过期后会被丢弃。                 | 用于传输有时效性的数据。                                     |
  | **Liveliness**          | 用于监控发布者是否“存活”。                               | 可以检测发布者是否崩溃。<br />`AUTOMATIC`（默认，DDS 自动管理心跳）、`MANUAL_BY_TOPIC`（用户手动发送心跳） |

  **关键点：Publisher和Subscriber的QoS必须兼容才能建立连接。** 规则通常是“要求高的服从要求低的”。例如，一个 `RELIABLE` 的Subscriber无法连接到一个 `BEST_EFFORT` 的Publisher，但反过来可以。

#### 3. 自动发现与动态匹配

DDS 节点启动后，会通过**Discovery（发现）** 机制自动寻找同一域内的其他节点，并交换 “发布 / 订阅的 Topic 及 QoS” 信息。当发布者和订阅者的 Topic、数据类型、QoS 兼容时，DDS 会自动建立通信连接（无需手动配置 IP / 端口）。

- 发现机制依赖底层的 UDP 多播（默认）或单播，确保节点动态加入 / 退出时的灵活性。

1. **参与者发现**：当一个节点启动时，它的 `DomainParticipant` 会通过组播（默认）在网络中宣告自己的存在。其他节点的 `DomainParticipant` 会收到这个信息，从而知道彼此的存在。
2. **端点发现**：当一个节点创建了 `Publisher/DataWriter` 或 `Subscriber/DataReader`，它会将这一信息（包括它关心的Topic、数据类型、QoS等）广播出去。
3. **匹配**：当一个 `DataWriter` 和一个 `DataReader` 的以下属性匹配时，它们就建立了连接：
   - **相同的DDS域**。
   - **相同的Topic名称**。
   - **兼容的数据类型**。
   - **兼容的QoS策略**。

这个发现过程完成后，数据就可以直接在 `DataWriter` 和 `DataReader` 之间传输，**绕过了任何中心节点**。

#### 4. 数据序列化与传输

DDS 传输的数据需经过序列化（转为二进制）和反序列化（恢复为对象）。ROS 2 中使用**IDL（Interface Definition Language）** 定义数据类型（`.msg`文件会被编译为 IDL），并通过代码生成工具（如`rosidl`）生成序列化 / 反序列化代码，确保跨语言（C++、Python 等）和跨平台兼容性。

#### 5.**实时发布订阅协议**

DDS的底层通信协议是**RTPS**。RTPS是DDS的Wire Protocol，它定义了数据如何在网络上传输。

- **优势**：RTPS是标准协议，允许不同供应商的DDS实现相互通信。
- **传输支持**：RTPS可以运行在多种传输层上，如UDP、TCP、共享内存等。ROS 2主要使用UDP，因为它开销小、延迟低，非常适合实时系统。



### DDS在ROS 2中的用法

ROS 2并没有直接与某一个DDS实现绑定，而是通过一个抽象层——**RMW**——来集成不同的DDS实现。

- **RMW**：提供了一套统一的C API。所有ROS 2的客户端库（如rclcpp, rclpy）都通过RMW接口与底层的DDS通信。
- **DDS供应商**：是实现DDS标准的商业或开源库，例如：
  - **Cyclone DDS**： 轻量级、高性能，是ROS 2推荐的默认实现。
  - **Fast DDS**： 功能全面，以前是默认实现。
  - **Connext DDS**： 功能强大、商业级，提供最好的实时性和工具链支持（需要许可证）。

**如何选择？** 在编译ROS 2或运行程序时，通过设置环境变量 `RMW_IMPLEMENTATION` 来指定。

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# 或者
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

#### 实践指南与技巧

> 在ROS2中，默认的QoS是RELIABLE，KEEP_LAST，深度10。

1. **选择正确的DDS实现**：
   - 对于大多数应用，**Cyclone DDS** 是一个安全、高性能的选择。
   - 如果你需要高级功能（如安全加密、复杂的路由配置）或处于严格的商业环境，考虑 **Connext DDS**。
   - 使用 `ros2 doctor --report` 来检查你当前使用的RMW实现。
2. **明智地使用QoS**：
   - **传感器数据流**：`BEST_EFFORT` + `VOLATILE` + 合适的 `depth`。这能最大化减少延迟。
   - **控制指令**：`RELIABLE` + `VOLATILE`。确保每个指令都能送达。
   - **静态数据（如地图、TF）**：`RELIABLE` + `TRANSIENT_LOCAL`。确保新节点能立即获取数据。
   - 不要滥用 `RELIABLE`，因为它会带来重传开销，增加延迟。
3. **调试DDS问题**：
   - **节点无法发现**：检查是否在**同一个DDS域**（默认是0）。可以通过环境变量 `ROS_DOMAIN_ID` 来隔离不同机器人或团队的网络。
   - **QoS不匹配**：使用 `ros2 topic info -v <topic_name>` 查看Publisher和Subscriber的详细QoS配置，确认它们是否兼容。
   - **网络配置**：DDS发现默认使用组播。如果机器人之间在多机通信时无法发现，可能需要配置防火墙或使用单播。





## References

- [Github eProsima Fast DDS](https://github.com/eProsima/Fast-DDS)
- [offical eProsima](https://www.eprosima.com/)
- [FastDDS doc with eProsima ](https://fast-dds.docs.eprosima.com/en/latest/)
- [Design ROS on DDS](https://design.ros2.org/articles/ros_on_dds.html)
- [CSDN  ROS2 DDS中间件（图文并茂+超详细）](https://blog.csdn.net/weixin_39939185/article/details/145865272)
- [性能优化](https://code-bai.com/2025/01/19/ros2-performance-optimization/)





