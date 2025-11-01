---
created: 2025-10-17T16:20:28
tags:

---

开发机器人时，可以类比人类的工作方式：大脑、脊髓、神经信号、神经元、神经网络等等概念。



## 机器人操作系统(ROS2)



### Node

节点是ROS2中的基本执行单元。每个节点通常负责一个单一的、模块化的功能。例如，一个节点可以控制激光雷达，另一个节点可以处理激光雷达的数据，第三个节点可以负责运动规划。



讲清楚节点是做什么的？

>  每个节点都是一个可以**独立运行的可执行文件**，执行某些具体的任务。

节点与节点是怎样通信的？

> 通过 topic、service、action 来通信。



特点

1. 每个节点的名称具有唯一性。
2. 节点是独立的，可以运行在同一台机器上，也可以分布在不同的机器上。
3. 一个可执行文件中包含一个或多个节点
4. 每个节点都可以发布或订阅话题，也可以提供或使用服务(service)。





### Topic

toopic 是节点之间交换信息的一种通信机制。这种通信是单向的、异步的。发布者（Publisher）node 将消息发布到 topic，订阅者（Subscriber）node 从 topic 订阅消息。

特点

- 单向通信：数据从发布者流向订阅者。
- 异步：发布者和订阅者不需要同时运行，也不需要知道彼此的存在。
- 多对多：多个发布者和多个订阅者可以同时使用同一个话题。

topic 通信接口的定义使用的是 `.msg`文件，由于是单向传输，只需要描述传输的每一帧数据是什么就行。

`xxx.msg`

```
int32 x
int32 y
```



### Service

服务是节点之间另一种通信机制，这种通信是双向的、同步的。它采用请求(reruest-reponse)-响应模型：一个客户端（Client）节点发送请求，然后等待服务器（Server）节点处理请求并返回响应。例如请求一个路径规划服务。

特点

- 双向通信：包括请求和响应。
- 同步：客户端发送请求后会阻塞，直到收到响应（当然，ROS2也支持异步服务调用）。
- 一对多：一个 service server 可以接受多个客户端的请求，但每个请求是串行处理的（除非服务器内部实现多线程）。

service 通信接口的定义使用的是 `.srv` 文件，包含请求和应答两部分定义，通过中间的“---”区分。

`xxx.srv`

```
# request
int64 a
int64 b

---
# response
int64 sum
```



### Action

Action是ROS 2中用于处理**长时间运行、可抢占、有反馈**的任务的通信机制。它采用**客户端-服务器**模式，但比Service更复杂。

**Action由三部分组成：**

1. **Goal（目标）**：客户端发送给服务端的任务目标（例如：移动到某个位置）。
2. **Feedback（反馈）**：服务端在执行过程中定期发送的进度更新（例如：已移动50%）。
3. **Result（结果）**：任务完成后发送的最终结果（例如：成功到达或失败原因）。

action 用于描述机器人的运动过程。比如：

客户端发送一个运动的目标，想让机器人动起来，服务器端收到之后，就开始控制机器人运动，一边运动，一边反馈当前的状态，如果是一个导航动作，这个反馈可能是当前所处的坐标，如果是机械臂抓取，这个反馈可能又是机械臂的实时姿态。当运动执行结束后，服务器再反馈一个动作结束的信息。整个通信过程就此结束。

**特点**

- ✅ **长时间运行**：任务可能需要几秒、几分钟甚至更长时间。
- ✅ **可抢占**：客户端可以随时取消正在执行的任务。
- ✅ **有进度反馈**：服务端定期向客户端发送进度更新。
- ✅ **双向通信**：客户端发送目标，服务端返回结果和反馈。



`action`通信接口的定义使用的是 `.action` 文件。

`xxx.action`

```
# goal
bool enable

---
# result
bool finish

# feedback
int state
```



### Parameter

Parameter是ROS 2中用于动态配置节点(node)的键值对。它们可以在节点运行时动态修改，而不需要重新编译代码。

实际例子

```bash
节点启动时读取参数：
- 最大速度：1.0 m/s
- 机器人名称："my_robot"
- 使用模拟器：True

在运行时，用户可以通过命令行或图形界面工具动态修改这些参数。
```

**特点：**

- ✅ **动态配置**：可以在节点运行时改变参数值。比如：PID增益、速度限制
- ✅ **类型安全**：参数有明确的类型（整数、浮点数、字符串、布尔等）。
- ✅ **可动态重新配置**：节点可以响应参数变化并调整行为。
- ✅ **支持多种来源**：可以从YAML文件、命令行或参数服务器设置。

**参数使用方式**

1. **声明参数**：节点在启动时声明它需要哪些参数。
2. **设置参数**：用户通过命令行或配置文件设置参数值。
3. **读取参数**：节点在运行过程中读取参数值。
4. **监视参数变化**：节点可以设置回调函数来响应参数变化。



总结：**Topic用于数据流，Service用于即时操作，Action用于长期任务，Parameter用于配置。** 





### Executor

**执行器（Executor）** 它负责让节点“活”起来，并决定节点如何响应外部世界。

#### 核心思想：事件循环

在ROS2中，节点可以通过订阅者、计时器、服务服务器、动作服务器等与外部通信。这些组件在创建后，并不会自动运行。它们只是在等待，就像一堆待办事项清单。

**执行器** 就是一个不断循环的进程，它的工作就是不停地检查这个“待办事项清单”，看看有没有新的事情需要处理。例如：

- 有没有收到新的消息？（订阅者）
- 定时器的时间到了吗？（计时器）
- 有没有收到服务请求？（服务服务器）
- 有没有收到新的动作目标？（动作服务器）

一旦执行器发现某个事件就绪了，它就会调用相应的回调函数来处理它。这个循环过程就是 **事件循环**。

------

#### 为什么需要执行器？

没有执行器，你的节点代码会像下面这样，什么也做不了：

```python
# 伪代码：没有执行器的情况
rospy.init_node('my_node')
sub = rospy.Subscriber('chatter', String, callback_function)
timer = rospy.Timer(rospy.Duration(1.0), timer_callback)

# 程序执行到这里就结束了，永远不会调用callback_function或timer_callback
print("Node created, but doing nothing. Exiting.")
```

执行器的作用就是让程序**停留**在事件循环中，保持节点的活性，使其能够持续响应。

```python
# 伪代码：有执行器的情况
rospy.init_node('my_node')
sub = rospy.Subscriber('chatter', String, callback_function)
timer = rospy.Timer(rospy.Duration(1.0), timer_callback)

executor = SingleThreadedExecutor()
executor.add_node(my_node)
executor.spin() # 程序在这里进入无限循环，处理事件，永远不会退出（除非被中断）
```

#### Callback Group

当使用 `MultiThreadedExecutor` 时，你可以通过 **回调组（Callback Group）** 来更精细地控制回调的执行策略。主要有两种类型：

- **Mutually Exclusive**：组内的回调**不能**同时执行。即使有多个线程，这个组也像是一个“小单线程”，用于保护共享资源。
- **Reentrant**：组内的回调**可以**同时执行。这是默认行为。

通过将不同的回调分配到不同的组，你可以实现复杂的并发控制，而无需在回调函数内部手动加锁。

### DDS

- [Github eProsima Fast DDS](https://github.com/eProsima/Fast-DDS)
- [offical eProsima](https://www.eprosima.com/)
- [FastDDS doc with eProsima ](https://fast-dds.docs.eprosima.com/en/latest/)
- [Design ROS on DDS](https://design.ros2.org/articles/ros_on_dds.html)
- [CSDN  ROS2 DDS中间件（图文并茂+超详细）](https://blog.csdn.net/weixin_39939185/article/details/145865272)
- [性能优化](https://code-bai.com/2025/01/19/ros2-performance-optimization/)



DDS 是 ROS2 的 底层通信机制。它是一种通信标准和一套API 定义。ROS2 默认采用的 DDS 实现为 eProsima FastDDS。

DDS的全称是**Data Distribution Service**，也就是**数据分发服务**，2004年由**对象管理组织OMG**发布和维护，是一套专门为**实时系统**设计的**数据分发/订阅标准**，最早应用于美国海军， 解决舰船复杂网络环境中大量软件升级的兼容性问题，现在已经成为强制标准。

DDS强调**以数据为中心**，可以提供丰富的**服务质量策略**(QoS)，以保障数据进行实时、高效、灵活地分发，可满足各种分布式实时通信应用需求



QoS

- reliability
- history
- depth

可靠性(Reliability):RELIABLE确保所有数据被接收，适合关键传感器数据;BEST EFFORT追求速度，适合非关键的视频流

历史记录(History):KEEP LAST(n)仅保留最新n条数据;KEEP ALL保留所有数据(需谨慎使用内存)

生存时间(Liveliness):监控发布者状态，检测节点故障

截止时间(Deadline):指定数据更新的最大间隔，确保实时性







### 工作流程

1. 设置ROS2工作空间

2. 用 `ros2 create` 创建一个ROS2包。

   ```bash
   # 创建ROS2包
   ros2 pkg create --build-type ament_cmake --node-name hello_world cpp_hello_world
   ```

3. 编写一个发布者节点

4. 修改`CMakeLists.txt` 和 `package.xml`

   ```cmake
   cmake_minimum_required(VERSION 3.8)
   project(cpp_hello_world)
   
   # 查找依赖
   find_package(ament_cmake REQUIRED)
   find_package(rclcpp REQUIRED)
   
   # 创建可执行文件
   add_executable(hello_world src/hello_world.cpp)
   # 添加依赖
   ament_target_dependencies(hello_world rclcpp)
   
   # 安装可执行文件
   install(TARGETS
     hello_world
     DESTINATION lib/${PROJECT_NAME}
   )
   
   # 导出依赖
   ament_export_dependencies(rclcpp)
   
   # 生成包配置
   ament_package()
   ```

5. 编译包

   ```bash
   cd ~/ros2_ws
   
   # 安装依赖（首次需要）
   rosdep install -i --from-path src --rosdistro humble -y
   
   # 编译包
   colcon build --packages-select cpp_hello_world
   
   # 加载工作空间环境
   source install/setup.bash
   ```

6. 运行节点

   ```bash
   # 方法1：直接运行
   ros2 run cpp_hello_world hello_world
   
   # 方法2：启动并在后台运行
   ros2 run cpp_hello_world hello_world &
   ```

7. 验证节点运行

   ```bash
   # 查看运行的节点
   ros2 node list
   
   # 查看节点信息
   ros2 node info /hello_world_node
   
   # 查看节点输出
   ros2 topic echo /rosout
   ```

   



### 工具

#### colcon



#### ament



#### Gazebo

用于仿真。



#### Rviz

一种可视化工具。



#### qrt



#### launch

ROS 系统中多 node 启动与 配置的一种脚本。





#### URDF

机器人建模方法，用来描述机器人外观、性能个方面的属性。

机器人一般是由**硬件结构、驱动系统、传感器系统、控制系统**四大部分组成。

- 硬件结构就是底盘、外壳、电机等实打实可以看到的设备；
- 驱动系统就是可以驱使这些设备正常使用的装置，比如电机的驱动器，电源管理系统等；
- 传感系统包括电机上的编码器、板载的IMU、安装的摄像头、雷达等等，便于机器人感知自己的状态和外部的环境；
- 控制系统就是我们开发过程的主要载体了，一般是树莓派、电脑等计算平台，以及里边的操作系统和应用软件。

机器人建模的过程，其实就是按照类似的思路，通过建模语言，把机器人每一个部分都描述清楚，再组合起来的过程。

ROS中的建模方法叫做**URDF(Unified Robot Description Format)**，全称是**统一机器人描述格式**，不仅可以清晰描述机器人自身的模型，还可以描述机器人的外部环境。





#### rosbag2

用于数据记录。

默认使用 **SQLite3** 数据库（`.db3` 文件），但也支持其他格式（如 MCAP）。

将数据以发布订阅的方式发布到 指定的 topic。



**核心功能**

- **录制**：监听一个或多个指定的 ROS 2 话题，并将发布到这些话题上的所有消息序列化后写入磁盘文件。
- **回放**：读取之前录制的文件，并按照时间顺序将消息重新发布到对应的话题上，模拟出数据当初产生的场景。
- **信息查询**：查看录制文件的内容，例如包含了哪些话题、消息类型、录制时长等信息。







#### ROS2 命令

##### run

```bash
ros2 run <package_name> <executable_name>
```



##### package

创建指令

```bash
# 创建包含依赖的包
ros2 pkg create --build-type ament_cmake \
                --node-name my_node \
                --dependencies rclcpp std_msgs geometry_msgs \
                my_advanced_pkg

# 创建库包（无节点）
ros2 pkg create --build-type ament_cmake my_library_pkg

# 创建包含特定许可证的包
ros2 pkg create --build-type ament_cmake \
                --node-name my_node \
                --license Apache-2.0 \
                my_licensed_pkg
```







## ROS2  core packages

**ROS Index**: https://index.ros.org/



### rcl

ros doc rcp API: https://docs.ros.org/en/jazzy/p/rcl/



### rclcpp

rclcpp c++ API: https://docs.ros.org/en/jazzy/p/rclcpp/generated/index.html

### rmw

ros rmw API: https://docs.ros.org/en/jazzy/p/rmw/



### lifecycle

- [offical lifecycle API](https://docs.ros.org/en/jazzy/p/lifecycle/)
- [manage d-life cycle nodes in ROS 2](https://design.ros2.org/articles/node_lifecycle.html)

问题：

1. 节点管理问题：节点启动顺序或状态切换不稳定。





Lifecycle Node 的核心状态机包含以下几个主要状态(primary states)：

1. **`Unconfigured`**：
   - **描述**：节点的“出生”状态。节点刚被启动，但尚未进行任何配置。它不能执行任何功能。
   - **类比**：机器人刚通电，但所有软件都还没加载参数。
2. **`Inactive`**：
   - **描述**：节点已配置完成（例如，已从参数服务器读取了所有参数），但尚未被激活。此时，它**没有**创建任何 Publisher、Subscriber、Service 等通信对象，因此不消耗网络带宽和计算资源。
   - **类比**：机器人的手臂驱动程序已经加载了配置（如长度、关节限制），但电机尚未上电，手臂不会运动。
3. **`Active`**：
   - **描述**：节点的“工作”状态。此时，节点已经创建了所有必要的 Publisher、Subscriber 等，开始执行其核心逻辑（如发布传感器数据、处理数据、响应服务请求）。
   - **类比**：机器人手臂电机上电，控制器开始运行，可以接收指令并运动。
4. **`Finalized`**：
   - **描述**：节点的“死亡”状态。节点正在或已经清理了所有资源，即将关闭。这是一个不可逆的终点状态。
   - **类比**：机器人关闭，所有程序退出。

此外，还有一些**过渡状态**，如 `Configuring`, `Activating`, `Deactivating`, `CleaningUp`, `ErrorProcessing`。这些状态表示节点正在执行从一个主要状态切换到另一个主要状态所需的操作。









#### rclcpp::spin(node)

`rclcpp::spin(node)` 是ROS2节点运行的核心机制

- 🎯 **作用**：保持节点活跃，处理所有异步事件
- ⚡ **必要性**：没有它，所有回调函数都不会执行
- 🔄 **行为**：阻塞当前线程，持续处理事件
- 🛑 **退出**：通过信号或显式关闭来停止

理解 `spin` 是编写功能完整的ROS2节点的关键！这是节点从"静态代码"变成"动态系统"的魔法所在。 ✨



#### 底层原理

`rclcpp::spin()` 实际上做了这些事情：

1. **进入事件循环**
2. **检查以下事件**：
   - 定时器是否到期
   - 是否有新消息到达
   - 是否有服务请求
   - 是否有动作目标
3. **调用对应的回调函数**
4. **重复循环**



#### 什么时候 spin 会结束？

```cpp
rclcpp::spin(node);  // 这个调用会阻塞，直到：

// 以下情况会退出 spin：
// 1. 收到 Ctrl+C 信号
// 2. 调用 rclcpp::shutdown()
// 3. 节点被显式销毁
// 4. 使用 spin_some() 或带超时的 spin
```

#### spin 的几种用法

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

#### spin 选择建议

1. **默认情况下**，如果节点简单，回调函数短小且不需要并行，使用单线程。
2. **当节点有多个回调，且其中一个或多个回调可能阻塞时**，使用多线程。
3. **如果节点有多个回调，且它们之间需要共享数据**，则必须注意线程安全。如果使用多线程，需要适当的同步机制（如互斥锁）。如果使用单线程，则无需担心。

### rclpy





### node

- QoS

  

#### timer

- `create_wall_timer()`： 是一个用于创建周期性定时器的工具，但它**不依赖于 ROS 系统的仿真时间或系统时间**，而是基于“挂钟时间”。它通常用于需要稳定、真实时间间隔的任务，例如控制循环、状态监测、或与外部非ROS系统交互。



### parameter

### publisher

### sub

### std_msgs

在工程中使用`std_msgs`的典型场景和示例：

1. **导入消息类型**：在编写节点时，首先需要导入所需的`std_msgs`消息类型。
2. **发布消息**：创建一个发布者，用于发布某种`std_msgs`类型的消息。
3. **订阅消息**：创建一个订阅者，接收并处理某种`std_msgs`类型的消息。
4. **自定义消息组合**：在自定义消息中，可以使用`std_msgs`中的基本类型作为组成部分。



### rosidl

[github ros2 idl](https://github.com/ros2/rosidl)





### urdf







## Hardware

1. 应用处理器。用层可能只要毫秒级响应

   一块像**树莓派**、**英伟达Jetson** 或者普通**电脑主板**一样的东西。运行Linux系统。

   1. 多核CPU：一般是 ARM 架构。
   2. GPU
   3. RAM
   4. Memory

2. 运动控制器。运动控制需要微秒级的响应

   一块嵌入在机器人身体里的**专用电路板**，比如基于 **STM32**、**DSP** 或 **FPGA** 的板卡。

   1. MCU：ST系列
   2. Flash
   3. RAM：容量有限。

3. 底盘

   1. 编码器电机：PID算法。
   2. 摄像头：USB摄像头、
   3. 方向轮（可选）

4. 激光雷达

   一般观察指标：扫描频率、扫描角度、测距频率

5. 姿态传感器
   1. 加速度
   2. 陀螺仪
   3. 角度输出（带卡尔曼滤波）

6. 电池



## References

- [Offical ROS2](https://ros.org/)
- [ROS2 Document with jazzy](https://docs.ros.org/en/jazzy/index.html)
- [ROS on DDS](https://design.ros2.org/articles/ros_on_dds.html)
- [ros2_control documentation](https://control.ros.org/rolling/index.html)
- [ROS2 Design Articles](https://design.ros2.org/)
- [offical moveit](https://moveit.ros.org/): 路径规划
- [offical Nav2](https://navigation.ros.org/)
- [VSCode, Docker, and ROS2](https://www.allisonthackston.com/articles/vscode-docker-ros2.html)


---

- [古月居 图书资源](https://book.guyuehome.com/)
- [OriginBot智能机器人开源套件](http://originbot.org/index.html)
- [动手学ROS2](http://fishros.com/d2lros2/)
- [鱼香ROS机器人](http://fishros.com/)
- [宇树具身智能](https://www.unifolm.com/)
- [Lumina 具身智能社区](https://lumina-embodied.ai/)
- [Github 开源的具身智能社区](https://github.com/TianxingChen/Embodied-AI-Guide)

















































