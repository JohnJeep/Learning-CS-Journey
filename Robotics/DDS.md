<!--

 * @Author: JohnJeep
 * @Date: 2025-10-26 16:46:49
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-05-31 18:48:27
 * @Description: DDS fundamental concepts and usage in ROS 2
 * Copyright (c) 2025 by John Jeep, All Rights Reserved. 
-->

# 1. Introduction to DDS

DDS 的全称是**Data Distribution Service**，也就是**数据分发服务**，2004 年由**对象管理组织
OMG**发布和维护，是一套专门为**实时系统**设计的**数据分发/订阅标准**，最早应用于美国海军，
解决舰船复杂网络环境中大量软件升级的兼容性问题，现在已经成为强制标准。

DDS 强调**以数据为中心**，可以提供丰富的**服务质量策略**(QoS)，以保障数据进行实时、高效、灵活地分发，可满足各种分布式
实时通信应用需求

DDS 是 ROS2 的 底层通信机制，负责节点间的数据传输、发现和通信管理。它是一种通信标准和一套 API 定义。ROS2 采用
eProsima FastDDS 为默认的 DDS。


# 2. DDS 核心原理

DDS 是一个面向实时、分布式系统的中间件标准（由 OMG 组织定义），核心目标是高效、可靠地实现动态节点间的数据共享。其核心
原理可总结为以下几点：


## 2.1. 以 “数据为中心” 的通信模型

DDS 与 ROS 1 的 “以节点为中心” 不同，它将**数据本身**作为通信的核心，节点通过 “发布 / 订阅”
数据主题（Topic）进行交互，无需关心数据的发送方或接收方是谁。

- 节点只需声明自己关心的数据（订阅）或产生的数据（发布），DDS 会自动处理数据的路由和分发。
- 这种模型更适合动态系统（如节点随时加入 / 退出），无需预先配置通信拓扑。

一句话概括：**DDS 是一个以 数据 为中心的 发布订阅模型。**


## 2.2. 关键概念：Domain、Topic、QoS

- **Domain（域）**：DDS 通过 “域” 隔离不同的通信群体，只有处于同一域（Domain ID
  相同）的节点才能相互发现和通信。例如，ROS 2 默认使用`Domain
  ID=0`，若需隔离多个系统，可修改该值（如`export ROS_DOMAIN_ID=1`）。

- **Topic（主题）**：数据的逻辑标识，由 “名称” 和 “数据类型” 共同定义（两者缺一不可）。例如，ROS 2
  中的`/chatter`主题，数据类型为`std_msgs/msg/String`，只有发布和订阅相同名称 +
  类型的节点才能通信。

- **QoS（服务质量）**
在 ROS 2 中，QoS（Quality of Service，服务质量）策略是控制节点间数据传输行为的核心机制，直接影响通信的可靠性、实时性、
数据持久性等。配置和使用 QoS
需要结合具体场景（如传感器数据、控制指令、日志信息等），确保数据传输符合需求。

QoS 是 DDS 的灵魂，也是你作为开发者必须掌握的工具。它允许你精确控制通信行为。在 ROS 2 中，你可以在创建 `Publisher` 或
`Subscriber` 时指定 QoS。

  **常见且重要的 QoS 策略：**

  | QoS 策略                 | 功能描述                                                 | 常用场景                                                     |
  | :---------------------- | :------------------------------------------------------- | :----------------------------------------------------------- |
  | **Reliability**(可靠性) | 确保数据可靠交付。                                       |                                                              |
  | `RELIABLE`              | 保证数据按顺序送达，丢失会重传。                         | **命令控制**（如 `/cmd_vel`），**关键状态**。                |
  | `BEST_EFFORT`           | 尽最大努力交付，可能丢失数据，但延迟更低。               | **高频传感器数据**（如激光雷达、摄像头），丢失一帧无所谓。   |
  | **Durability**(持久性)  | 控制历史数据的持久化。                                   |                                                              |
  | `VOLATILE`              | 不保存历史数据。新加入的订阅者收不到之前的数据。         | 大多数传感器数据。                                           |
  | `TRANSIENT_LOCAL`       | 发布者为最后 N 个样本保留历史。新订阅者能收到最近的数据。  | **地图**、**参数配置**、**静态变换**。新节点启动后能立刻获取到地图。 |
  | **History**(历史记录)   | 当处理速度跟不上发布速度时，如何管理缓存中的数据。       |                                                              |
  | `KEEP_LAST`             | 只保留最新的 N 个样本。                                    | 最常见的选择。需要指定 `depth`。                             |
  | `KEEP_ALL`              | 保留所有样本，直到被订阅者取走。                         | 与 `RELIABLE` 配合，确保不丢任何数据。                       |
  | **Depth**               | 与 `KEEP_LAST` 配合，定义历史队列的大小。                | 根据系统处理能力和数据频率设定。例如 `depth=10`。            |
  | **Deadline**(截止时间)  | 指定数据更新的最大间隔，超过则触发回调（确保数据时效性） | 用于**时序监控**。如果发布者未在指定周期内发布数据，会触发回调。 |
  | **Lifespan**(生命周期)  | 数据在发布后的“有效期”，过期后会被丢弃。                 | 用于传输有时效性的数据。                                     |
  | **Liveliness**          | 用于监控发布者是否“存活”。                               | 可以检测发布者是否崩溃。<br />`AUTOMATIC`（默认，DDS 自动管理心跳）、`MANUAL_BY_TOPIC`（用户手动发送心跳） |

  **关键点：Publisher 和 Subscriber 的 QoS 必须兼容才能建立连接。** 规则通常是“要求高的服从要求低的”。例如，一个
  `RELIABLE` 的 Subscriber 无法连接到一个
  `BEST_EFFORT` 的 Publisher，但反过来可以。


## 2.3. 自动发现与动态匹配

DDS 节点启动后，会通过**Discovery（发现）** 机制自动寻找同一域内的其他节点，并交换 “发布 / 订阅的 Topic 及 QoS”
信息。当发布者和订阅者的 Topic、数据类型、QoS 兼容时，DDS
会自动建立通信连接（无需手动配置 IP / 端口）。

- 发现机制依赖底层的 UDP 多播（默认）或单播，确保节点动态加入 / 退出时的灵活性。

1. **参与者发现**：当一个节点启动时，它的 `DomainParticipant` 会通过组播（默认）在网络中宣告自己的存在。其他节点的
   `DomainParticipant` 会收到这个信息，从而知道彼此的存在。
2. **端点发现**：当一个节点创建了 `Publisher/DataWriter` 或 `Subscriber/DataReader`，它会将这一信息（包括它关心的
   Topic、数据类型、QoS 等）广播出去。
3. **匹配**：当一个 `DataWriter` 和一个 `DataReader` 的以下属性匹配时，它们就建立了连接：
   - **相同的 DDS 域**。
   - **相同的 Topic 名称**。
   - **兼容的数据类型**。
   - **兼容的 QoS 策略**。

这个发现过程完成后，数据就可以直接在 `DataWriter` 和 `DataReader` 之间传输，**绕过了任何中心节点**。


## 2.4. 数据序列化与传输

DDS 传输的数据需经过序列化（转为二进制）和反序列化（恢复为对象）。ROS 2 中使用**IDL（Interface Definition
Language）** 定义数据类型（`.msg`文件会被编译为
IDL），并通过代码生成工具（如`rosidl`）生成序列化 / 反序列化代码，确保跨语言（C++、Python 等）和跨平台兼容性。


## 2.5. 5.实时发布订阅协议

DDS 的底层通信协议是**RTPS**。RTPS 是 DDS 的 Wire Protocol，它定义了数据如何在网络上传输。

- **优势**：RTPS 是标准协议，允许不同供应商的 DDS 实现相互通信。
- **传输支持**：RTPS 可以运行在多种传输层上，如 UDP、TCP、共享内存等。ROS 2 主要使用
  UDP，因为它开销小、延迟低，非常适合实时系统。



# 3. DDS 在 ROS 2 中的用法

ROS 2 并没有直接与某一个 DDS 实现绑定，而是通过一个抽象层——**RMW**——来集成不同的 DDS 实现。

- **RMW**：提供了一套统一的 C API。所有 ROS 2 的客户端库（如 rclcpp, rclpy）都通过 RMW 接口与底层的 DDS 通信。
- **DDS 供应商**：是实现 DDS 标准的商业或开源库，例如：
  - **Cyclone DDS**： 轻量级、高性能，是 ROS 2 推荐的默认实现。
  - **Fast DDS**： 功能全面，以前是默认实现。
  - **Connext DDS**： 功能强大、商业级，提供最好的实时性和工具链支持（需要许可证）。

**如何选择？** 在编译 ROS 2 或运行程序时，通过设置环境变量 `RMW_IMPLEMENTATION` 来指定。

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# 或者
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```


## 3.1. 实践指南与技巧

> 在 ROS2 中，默认的 QoS 是 RELIABLE，KEEP_LAST，深度 10。

1. **选择正确的 DDS 实现**：
   - 对于大多数应用，**Cyclone DDS** 是一个安全、高性能的选择。
   - 如果你需要高级功能（如安全加密、复杂的路由配置）或处于严格的商业环境，考虑 **Connext DDS**。
   - 使用 `ros2 doctor --report` 来检查你当前使用的 RMW 实现。
2. **明智地使用 QoS**：
   - **传感器数据流**：`BEST_EFFORT` + `VOLATILE` + 合适的 `depth`。这能最大化减少延迟。
   - **控制指令**：`RELIABLE` + `VOLATILE`。确保每个指令都能送达。
   - **静态数据（如地图、TF）**：`RELIABLE` + `TRANSIENT_LOCAL`。确保新节点能立即获取数据。
   - 不要滥用 `RELIABLE`，因为它会带来重传开销，增加延迟。
3. **调试 DDS 问题**：
   - **节点无法发现**：检查是否在**同一个 DDS 域**（默认是 0）。可以通过环境变量 `ROS_DOMAIN_ID`
     来隔离不同机器人或团队的网络。
   - **QoS 不匹配**：使用 `ros2 topic info -v <topic_name>` 查看 Publisher 和 Subscriber 的详细 QoS
     配置，确认它们是否兼容。
   - **网络配置**：DDS 发现默认使用组播。如果机器人之间在多机通信时无法发现，可能需要配置防火墙或使用单播。




# 4. FastDDS

FastDDS 是 eProsima 公司开发的一个高性能、开源的 DDS 实现，曾经是 ROS 2 的默认 DDS
实现。它提供了丰富的功能和工具支持，适用于各种实时通信场景。

**Fast DDS（eProsima）也有自己的零拷贝方案**，但走的是不同路线——它实现了自己的 **"Data-sharing delivery"** 机制（同样基于共享内存），而不是去集成 Iceoryx。所以严格来说 Fast DDS 生态里一般不需要 Iceoryx，因为它自带了功能相近的替代方案。



# 5. CycloneDDS

## 5.1. 什么是 Cyclone DDS

**Cyclone DDS** 是 Eclipse 基金会维护的一个开源 DDS（Data Distribution Service）实现，是 ROS2 官方支持的几个 DDS 中间件之一（另外常见的还有 **Fast DDS**，即原来的 Fast-RTPS，以及商业的 RTI Connext）。目前是 ROS 2 推荐的默认 DDS
实现。它以其高效的性能和良好的稳定性而受到广泛使用，特别适合资源受限的环境。

## 5.2. 它解决什么问题

DDS 本身是一个标准（规范），定义了发布-订阅通信模型该怎么工作，但具体"怎么实现"由各厂商/项目自己写。Cyclone DDS 就是其中一种具体实现——负责节点发现、序列化、网络传输、QoS 策略执行这些底层机制。

## 5.3. 特点

- **轻量、低延迟**：相比 Fast DDS，Cyclone DDS 以性能和资源占用小著称，这对嵌入式/资源受限场景（比如你做的 aarch64 交叉编译栈）是个优势。
- **符合 OMG DDS 标准**：遵循 Object Management Group 的 DDS 规范，理论上和其他 DDS 实现能互操作。
- **ROS2 的默认选项之一**：ROS2 从 Foxy/Galactic 之后的一些发行版把 Cyclone DDS 作为默认或推荐的 RMW（ROS Middleware）实现，可以通过设置环境变量 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` 来切换使用它。
- **C 语言实现的核心**：这点对你从 C++ 角度理解会比较直接——核心库是 C 写的，性能开销小，适合嵌入式部署，也是 micro-ROS 常搭配使用的 DDS 实现之一。

# 6. Iceoryx

**Iceoryx** 是一个开源的**进程间通信（IPC）中间件**，专门为**真零拷贝（true zero-copy）**、低延迟的进程间数据共享设计，最初由博世（Bosch）开发，后来贡献给了 Eclipse 基金会（和 Cyclone DDS 是"邻居"项目）。

## 6.1. 它解决的核心问题

传统的进程间通信（比如 socket、共享内存加锁、甚至标准 DDS 传输）在同一台机器上跨进程传数据时，往往要经历"序列化 → 拷贝到内核缓冲区 → 再拷贝到目标进程"这样的过程。对于机器人系统里常见的大数据（比如点云、图像帧），这种拷贝开销会很可观。

**Iceoryx 用共享内存 + 引用传递的方式，做到真正的零拷贝**：发送方把数据写入共享内存池，接收方拿到的是指向同一块内存的指针（而不是数据的副本）。本质上类似于用智能指针在同进程内传递大对象——只是这里跨越了进程边界。

## 6.2. 和 ROS2 / DDS 的关系

这是最容易搞混的地方，理清一下：

- Iceoryx **不是** DDS 的替代品，而是可以作为某些 DDS 实现的**传输层插件**。
- 比如 **Cyclone DDS** 和 **Fast DDS** 都支持通过 Iceoryx 作为"同主机内"的传输后端——当发布者和订阅者在同一台机器的不同进程里时，中间件会自动切换用 Iceoryx 走共享内存，而不是走本机回环网络栈；跨机器时仍然走正常的 DDS 网络传输（UDP等）。
- 在 ROS2 里，通常通过 `rmw_cyclonedds_cpp` 配合 Iceoryx 的 `RouDi`（Iceoryx 的守护进程，负责管理共享内存池）来实现。

## 6.3. 关键特点

- **RouDi 守护进程**：Iceoryx 需要一个中心化的守护进程（RouDi）在系统启动时运行，负责协调共享内存的分配，有点像一个轻量级的资源管理器。
- **面向大负载优化**：图像、点云、雷达数据这类大尺寸消息是它最大化收益的场景；小消息（比如一个 float）零拷贝的收益不明显，甚至可能因共享内存管理开销而不划算。
- **C++11 实现，面向嵌入式/实时**：设计上考虑了实时约束，避免动态内存分配等不确定延迟来源。

## 6.4. RouDi

**RouDi** 是 Iceoryx 的**中心化守护进程**，全称来自 "**Rou**ting and **Di**scovery"（路由与发现）。它是整个 Iceoryx 零拷贝共享内存机制能够运作的核心枢纽——**没有 RouDi 在后台跑着，Iceoryx 的发布订阅通信完全无法工作**。

### 6.4.1. RouDi 具体做什么

#### 6.4.1.1. 共享内存池的管理者

Iceoryx 的零拷贝依赖预先分配好的共享内存段。RouDi 在**启动时**根据配置文件（通常是 `roudi_config.toml` 之类）分配好一整块共享内存，并划分成不同大小的内存块（chunk）池。所有后续的发布者/订阅者进程都是从这个由 RouDi 管理的内存池里"借用"和"归还"内存块，而不是自己去 malloc。

从 C++ 角度理解很直接：RouDi 类似于一个**中心化的内存分配器 + 引用计数管理器**，只不过管理的是跨进程共享的内存，而不是进程内的堆。

#### 6.4.1.2. 服务发现

当一个发布者进程和一个订阅者进程各自启动、想要通信时，它们需要"找到彼此"。RouDi 维护着一张全局的 topic/服务注册表，新进程连接上来时向 RouDi 注册自己发布或订阅的内容，RouDi 负责撮合双方，让它们知道对方的存在——这一步类似于 ROS1 里 `roscm master` 的角色，但只负责发现和内存协调，不转发实际数据。

#### 6.4.1.3. 生命周期与健康监测

RouDi 会监控连接到它的各个应用进程是否还存活。如果某个发布者进程崩溃了，RouDi 能检测到并回收它持有的共享内存块，避免"僵尸"进程一直占着内存不释放（这在长期运行的机器人系统里很关键，否则内存泄漏会慢慢拖垮系统）。

### 6.4.2. 关键点：数据本身不经过 RouDi

这是最容易被误解的地方——**RouDi 不转发实际的消息数据**。它只负责"牵线搭桥"和"分内存"：

1. 发布者向 RouDi 申请一块共享内存
2. 发布者直接把数据写入这块内存
3. 发布者把这块内存的"引用"（本质是一个偏移量/句柄）发给订阅者
4. 订阅者直接从共享内存里读，全程数据本身**零拷贝、不经过 RouDi**

所以 RouDi 更像是"物业管理处"——负责分配车位（内存块）、登记谁跟谁认识（发现），但不负责搬东西（数据传输）。

### 6.4.3. 在实际部署场景里的影响

若在 aarch64 交叉编译栈里跑 Iceoryx（比如结合 Cyclone DDS 用于点云/图像零拷贝），需要注意：

- **RouDi 必须作为独立进程先于所有 Iceoryx 应用启动**，通常是 `iox-roudi` 这个可执行文件，你的 Docker 容器启动脚本或者 systemd service 里需要显式拉起它。
- **配置文件要提前规划好内存池大小**：RouDi 是静态预分配内存的，如果你的点云/图像数据尺寸变化大，需要在配置里预留足够大的 chunk，否则运行时申请失败会导致发布/订阅报错。
- **崩溃恢复**：如果 RouDi 本身崩溃，所有依赖它的 Iceoryx 通信都会中断，这是引入这套机制后需要考虑的新的单点故障——值得在你们的健康监测/看门狗机制里加上对 RouDi 的存活检测。



## 6.5. Iceoryx 与Cyclone DDS 结合

**Iceoryx 和 Cyclone DDS 的结合是目前最成熟、使用最广泛的组合**，也是 ROS2 生态里的事实标准搭配。

 **为什么是 Cyclone DDS**

1. **官方原生支持**：Cyclone DDS 从其 `cyclonedds` 项目本身就提供了对 Iceoryx 的集成支持，通过一个叫 **`Iceoryx PSMX`**（PSMX = Pub-Sub Message Exchange，一种插件接口）的机制，让 Cyclone DDS 在检测到发布者和订阅者处于同一台主机时，自动切换到 Iceoryx 的共享内存传输，而不用你手动改代码。

2. **同源生态**：Cyclone DDS 和 Iceoryx 都属于 Eclipse 基金会旗下项目（分别是 Eclipse Cyclone DDS 和 Eclipse iceoryx），社区维护和版本适配上协同度更高。

3. ROS2 层面的整合

   ：在 ROS2 中，这个组合通常表现为：

   - RMW 层用 `rmw_cyclonedds_cpp`
   - 底层传输启用 Iceoryx，需要额外跑 Iceoryx 的守护进程 `iox-roudi`
   - 配置上要在 Cyclone DDS 的 XML 配置文件里显式声明启用共享内存传输（`SharedMemory` 相关配置项）



# 7. Fast DDS vs Cyclone DDS 全面对比

|               | Fast DDS               | Cyclone DDS                                       |
| ------------- | ---------------------- | ------------------------------------------------- |
| 维护方        | eProsima（西班牙公司） | Eclipse 基金会                                    |
| 前身          | Fast-RTPS              | ADLINK 内部项目开源而来                           |
| 开发语言      | C++                    | C（核心），带 C++/C++11 绑定层                    |
| 开源协议      | Apache 2.0             | EPL 2.0                                           |
| ROS2 历史地位 | 长期默认 RMW 实现      | 部分发行版（如 Foxy 之后一些配置）的备选/默认之一 |

## 7.1. 架构与代码风格

**Fast DDS**：纯 C++ 实现，面向对象设计，功能非常全面——几乎实现了 DDS 标准里的所有可选特性（Content-Filtered Topics、Multi-Topic、各种 QoS 策略组合等）。代码库相对庞大。

**Cyclone DDS**：核心用 C 写，代码量更精简，聚焦在把核心的发布订阅、发现机制做得又快又稳。C++ API 是包装在 C 核心之上的一层薄封装。

## 7.2. 性能特点

这是两者最常被拿来比较的维度，但结论并不是一边倒：

- **Cyclone DDS** 在延迟和吞吐量上通常表现更好，尤其是在中小消息、高频率发布的场景下，社区和一些第三方 benchmark（比如 ROS2 官方的一些性能测试报告）里经常显示它延迟更低、抖动更小。这和它代码路径更短、内存分配更保守有关。
- **Fast DDS** 功能全面但相应地开销略高，不过 eProsima 近几年也在持续做性能优化，差距在缩小。
- 具体谁快取决于消息大小、QoS 配置、网络环境——**不要只信"哪个天生更快"这种笼统结论，实际部署前最好用你自己的负载做基准测试**。

## 7.3. 内存管理与实时性

- **Cyclone DDS** 在设计上更强调确定性内存分配（尽量避免运行时动态分配），更贴近硬实时/嵌入式场景的诉求，这也是为什么 micro-ROS 和资源受限设备上它更常被选用。
- **Fast DDS** 也支持静态内存分配模式（通过配置 History/Resource Limits QoS），但默认配置下动态分配更常见，需要额外调优才能达到类似的确定性。

## 7.4. 零拷贝支持

- **Fast DDS**：自带 **Data-Sharing Delivery** 机制，同主机内直接共享内存，不需要额外中间件。
- **Cyclone DDS**：本身不内置完整零拷贝，需要集成外部的 **Iceoryx** 才能实现同主机零拷贝。

## 7.5. 生态与工具链

- **Fast DDS** 提供了比较成熟的图形化监控工具（**Fast DDS Monitor**）和 Discovery Server（可选的、不依赖纯 P2P 多播发现的发现服务模式，适合大规模网络或多播受限的环境，比如云端部署）。
- **Cyclone DDS** 工具链相对轻量，但也提供了 `cyclonedds-tools` 里的一些调试命令（比如 `ddsperf` 用来测吞吐延迟），社区更偏"够用就好"。

## 7.6. 文档与社区支持

- **Fast DDS**：作为 ROS2 长期的默认实现，文档积累最丰富，遇到问题网上能查到的案例最多，对新手更友好。
- **Cyclone DDS**：文档相对精简，但核心开发者活跃，issue 响应速度不错，性能相关的深入讨论质量高。

---

## 7.7. 选 Fast DDS 的情况

- **默认场景 / 没有特殊性能诉求**：ROS2 的开箱默认配置，生态成熟，遇到问题容易查到解决方案。
- **需要跨广域网/云端发现**：Fast DDS 的 Discovery Server 模式对多播受限或者需要中心化发现管理的网络环境（比如跨多个子网、云端机器人集群）更友好。
- **需要功能完整性**：如果你要用到一些冷门但标准里定义的 DDS 特性（Content Filtered Topic 等），Fast DDS 覆盖更全。
- **团队对调试工具链有依赖**：需要图形化监控、可视化 topic 流量的场景。

## 7.8. 选 Cyclone DDS 的情况

- **嵌入式 / 资源受限设备**：若在 aarch64 上做交叉编译栈，目标平台内存、CPU 都紧张，Cyclone DDS 更轻量的内存足迹是个明显优势。
- **对延迟和抖动敏感的实时控制回路**：如果对延迟稳定性要求高，Cyclone DDS 通常表现更稳。
- **需要零拷贝但愿意多引入一个 Iceoryx 依赖**：如果点云、图像这类大负载是主要瓶颈，Cyclone DDS + Iceoryx 这套组合性能收益明显。
- **micro-ROS 场景**：如果链路上有需要跑在 MCU/裸机上的节点，Cyclone DDS 生态对 micro-ROS 支持更原生。



# 8. References

- [Github eProsima Fast DDS](https://github.com/eProsima/Fast-DDS)
- [offical eProsima](https://www.eprosima.com/)
- [FastDDS doc with eProsima ](https://fast-dds.docs.eprosima.com/en/latest/)
- [Design ROS on DDS](https://design.ros2.org/articles/ros_on_dds.html)
- https://github.com/eclipse-iceoryx/iceoryx

