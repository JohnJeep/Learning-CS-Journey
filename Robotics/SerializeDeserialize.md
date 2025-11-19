### 为什么ROS 2需要序列化/反序列化？

ROS 2是一个基于**匿名发布/订阅**模型的分布式系统。节点可能运行在不同的机器（不同的CPU架构）、不同的操作系统、甚至由不同编程语言（如C++、Python）编写。它们之间需要通过共享内存或网络（最常见）进行通信。

网络套接字和共享内存只能传输原始的**字节序列**，它们不理解`std_msgs::msg::String`或`sensor_msgs::msg::Image`这样的复杂数据结构。因此，必须有一个过程，在发送端将数据变成字节流，在接收端再将字节流变回数据。这就是序列化/反序列化存在的原因。

### ROS 2序列化/反序列化机制详解

ROS 2的序列化机制是高效、类型安全且与中间件解耦的。其核心设计可以概括为以下几点：

#### 1. 基于IDL和代码生成

ROS 2使用**OMG DDS Standard**的**接口定义语言**的一个子集来定义消息类型（`.msg`文件）。当你编译一个ROS 2工作空间时，`rosidl_generator`工具链会：

- **解析`.msg`文件**：读取消息的结构，包括字段名、类型（基本类型如`int32`、`float64`，或嵌套的其他消息类型）和数组长度。
- **生成特定语言的代码**：针对C++、Python等目标语言，生成对应的数据结构（如C++中的`struct`，Python中的类）以及**序列化器**和**反序列化器**。

这意味着，对于你定义的每一个消息类型，ROS 2都会为你自动生成一对专用的、高度优化的`serialize`和`deserialize`函数。

#### 2. CDR格式

ROS 2序列化后的字节流遵循**DDS的通用数据表示**格式。CDR(Common Data Representation)是一种标准化的、平台无关的二进制编码格式，它主要处理两个关键问题：

- **字节序**：CDR规定了使用**大端字节序**进行传输。发送方会检查自己的本地字节序（大端或小端），如果需要，会在序列化时进行转换。接收方在反序列化时也会进行同样的检查与转换。这确保了数据在不同架构的机器（如x86和ARM）之间能够正确解读。
- **内存对齐**：CDR会对数据进行填充，使其在字节流中自然对齐，这有助于在接收端高效地将数据直接映射到内存结构中。

#### 3. 序列化过程（以C++为例）

当你调用`publisher->publish(msg)`时，背后发生了：

1. **创建输出流**：分配一个内存缓冲区（如`std::vector<uint8_t>`）作为输出字节流。
2. **按字段序列化**：生成的序列化代码会按照消息定义的顺序，递归地将每个字段写入字节流。
   - 对于基本类型（如`int32`），直接将其字节表示（考虑字节序后）写入流。
   - 对于字符串，先写入字符串长度（作为一个`uint32`），再写入字符串的字符内容。
   - 对于数组，先写入数组长度，然后逐个序列化数组中的元素。
   - 对于嵌套的子消息，会递归调用该子消息的序列化方法。
3. **传递给DDS**：最终，这个充满数据的字节流被传递给底层的DDS中间件（如Cyclone DDS, Fast DDS）。DDS负责通过网络将这份“零件清单”可靠地、高效地分发给所有订阅者。

#### 4. 反序列化过程（以C++为例）

当订阅者收到数据时：

1. **DDS传递字节流**：DDS中间件将接收到的原始字节流传递给ROS 2客户端库（RCL）。
2. **创建空消息对象**：订阅者回调函数中通常会创建一个空的消息对象（例如 `auto msg = std::make_unique<sensor_msgs::msg::Image>()`）。
3. **按字段反序列化**：生成的反序列化代码会按照与序列化时**完全相同的顺序**，从字节流中读取数据来填充这个空对象。
   - 它知道第一个字段是`std_msgs/Header header`，于是开始递归反序列化`header`。
   - 接着它知道下一个是`uint32 height`，于是从流中读取4个字节，转换成本机字节序，并赋值给`msg->height`。
   - 如此继续，直到所有字段都被填充。
4. **触发回调**：现在，`msg`对象已经完全恢复了发送时的状态，随后它被传递给用户定义的订阅回调函数进行处理。



### 优化序列化/反序列化的方法

测试工具：在优化前先用`ros2 topic hz`和`ros2 topic bw`测量实际性能



#### 进程内通信(Intra-Process Communication)

当发布者和订阅者在同一个进程中时，ROS 2可以绕过DDS的序列化、网络传输和反序列化整个流程，直接在内存中传递消息对象的指针。

**实现方式：**

**a) 组件化节点**
这是ROS 2官方推荐的最佳实践。通过将多个节点编译到一个进程中，它们之间的通信会自动使用高效的进程内通信。

```makefile
// CMakeLists.txt
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 创建组件库
add_library(talker_component SHARED src/talker_component.cpp)
ament_target_dependencies(talker_component rclcpp std_msgs)

add_library(listener_component SHARED src/listener_component.cpp)
ament_target_dependencies(listener_component rclcpp std_msgs)

# 创建包含多个组件的可执行文件
add_executable(components_based src/components_based.cpp)
target_link_libraries(components_based talker_component listener_component)
ament_target_dependencies(components_based rclcpp)

install(TARGETS
  talker_component
  listener_component
  components_based
  DESTINATION lib/${PROJECT_NAME}
)
```

**b) 显式启用进程内通信**

即使在同一个进程中，也需要在创建Publisher和Subscriber时确保使用兼容的QoS策略，并且通过`NodeOptions`启用进程内通信。

```cpp
// 创建节点时启用
rclcpp::NodeOptions options;
options.use_intra_process_comms(true); // 关键设置！
auto node = std::make_shared<MyNode>(options);

// 创建发布者和订阅者
auto pub = node->create_publisher<std_msgs::msg::String>("topic", 10);
auto sub = node->create_subscription<std_msgs::msg::String>(
    "topic", 
    10, 
    [](std_msgs::msg::String::UniquePtr msg) { // 注意：使用UniquePtr
        // 处理消息
    });
```

**效果**：**完全消除**了进程内节点间的序列化/反序列化，网络延迟的开销，从而降低延迟和提高吞吐量。



#### 应用场景

通信延迟要求 < 1ms；数据吞吐量 > 100MB/s

1. **高频率、大数据量的通信**：例如，**相机驱动节点和图像处理节点**之间，如果图像数据很大，且需要高频率处理，进程内通信可以避免数据拷贝和网络延迟。
2. **实时性要求高的控制回路**：例如，机器人的关节控制器节点和状态估计节点，需要极低的延迟来保证控制的实时性。
3. **紧密耦合的功能模块**：例如，路径规划节点和运动学逆解节点，它们之间需要频繁交换数据，且逻辑上紧密相关，放在同一个进程中可以简化通信。
4. **资源受限的环境**：在计算资源有限的嵌入式系统中，通过合并节点减少进程数量，可以降低系统开销（如上下文切换和内存占用）。
5. **简化系统部署**：当多个节点由同一团队开发，且需要作为一个整体部署时，可以将它们编译成组件（Component），然后在同一个进程中运行，这样也便于管理。

**注意**：进程内通信的节点必须使用兼容的QoS策略，并且注意线程模型，避免一个节点的回调函数阻塞其他节点。



**对 Node 的要求：**

- ✅ **必须所有Publisher/Subscriber在同一个Node对象内**
- ✅ Node创建时必须启用intra-process：`options.use_intra_process_comms(true)`
- ✅ 推荐使用Composition（组件）模式
- ❌ 不能用于通过`ros2 run`启动的独立节点



#### 零拷贝传输(Zero-Copy Transports)

零拷贝也就是共享内存，是属于进程间通信(Inter-Process Communication)的一种。

当节点**分布在不同的进程**中时，发布者和订阅者通过**共享内存**传递数据。发布者将消息序列化到共享内存区域，订阅者直接从该区域访问数据，避免了在发布端序列化和订阅端反序列化时对消息内容进行内存拷贝。

ROS2 的底层中间件（默认是Fast DDS）可以配置为使用共享内存作为传输层，替代传统的 UDP/TCP 环路。

**实现方式：**

**a) 使用支持零拷贝的DDS实现**

- **Cyclone DDS**：对零拷贝有很好的支持。
- **Fast DDS**：也提供了共享内存传输插件。

**b) 配置DDS以启用共享内存**

需要在环境变量或XML配置中启用共享内存传输。

创建一个 XML 配置文件 (`fastdds_shm.xml`)，将其放在包的根目录。

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastrtps_profiles">
    <transport_descriptors>
        <!-- 定义共享内存传输器 -->
        <transport_descriptor>
            <transport_id>shm</transport_id>
            <type>SHM</type>
            <segment_size>134217728</segment_size> <!-- 128MB -->
        </transport_descriptor>
    </transport_descriptors>

    <participant profile_name="shm_participant" is_default_profile="true">
        <rtps>
            <userTransports>
                <transport_id>shm</transport_id>
            </userTransports>
            <useBuiltinTransports>false</useBuiltinTransports>
        </rtps>
    </participant>
</profiles>
```

**在运行程序前，设置环境变量来使用我们的配置文件：**

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/your/shm_ws/src/shm_demo/fastdds_shm.xml
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp # 确保使用 Fast DDS
```

**在一个终端运行发布者：**

```bash
ros2 run shm_demo publisher
```

**在另一个终端运行订阅者：**

```bash
ros2 run shm_demo subscriber
```

如果配置成功，你将看到它们正常通信。此时，它们之间的数据传输（尤其是对于大数据类型）将通过共享内存进行。

**验证共享内存是否生效**

你可以使用系统工具来检查共享内存的使用情况。

```bash
# 在 Linux 上，使用 `ipcs` 命令
ipcs -m

# 你应该会看到由 Fast DDS 创建的共享内存段
------ Shared Memory Segments --------
key        shmid      owner      perms      bytes      nattch     status
0x00000000 65536      your_user  700        134217728  2          dest
```

注意 `nattch` 字段，它显示有多少个进程连接到了这个共享内存段。在上面的例子中，`2` 表示发布者和订阅者都连接着。



**c) 使用LoanedMessage（C++）**
这是ROS 2 Foxy及以后版本提供的API，允许你"借用"DDS的底层内存来直接构造消息。

```cpp
// 发布端
auto loaned_msg = publisher->borrow_loaned_message();
// 直接填充loaned_msg的数据...
publisher->publish(std::move(loaned_msg));

// 订阅端需要配置为使用UniquePtr接收
```



**注意事项**：

- 消息类型必须是**平坦数据结构**（如`std_msgs::msg::String`、自定义的POD类型），不能包含指针或复杂容器。
- 需要DDS中间件的支持。
- 配置相对复杂。

**效果**：在不同进程间通信时，**消除了消息内容的内存拷贝**，但DDS头部信息可能仍需要序列化。



#### 应用场景

1. **模块化与解耦**：当不同节点由不同团队开发，或者功能相对独立时，分开进程可以降低耦合，便于独立开发和测试。
2. **容错与可靠性**：如果一个节点崩溃，它不会影响其他进程中的节点。例如，导航节点和感知节点分开，即使感知节点出现问题，导航节点可能仍能继续运行（尽管功能受限）。
3. **不同的运行要求**：某些节点可能需要不同的运行时环境，例如，有的节点需要GPU加速，有的节点需要实时内核，分开进程可以分别配置和优化。
4. **调试与监控**：单独进程可以更方便地使用工具（如GDB、Valgrind）进行调试，也可以独立监控每个节点的资源使用情况。
5. **利用多核性能**：不同进程可以并行运行在不同的CPU核心上，充分利用多核处理器的计算能力。
6. **第三方节点集成**：当集成第三方提供的节点时，通常以独立进程的形式运行，避免与自身节点相互干扰。
7. **安全隔离**：在某些安全关键系统中，可能希望将关键组件与非关键组件隔离在不同的进程中，以确保关键组件的正常运行。



**对 Node 的要求：**

- ✅ 节点可以在不同进程，但**必须在同一台主机**
- ✅ 需要使用支持共享内存的DDS实现（Fast DDS/Cyclone DDS）
- ✅ 需要正确配置DDS XML配置文件
- ✅ 需要设置环境变量指向配置文件
- ❌ 不能用于跨主机通信



#### 数据传输优化 - 减少传输次数

**核心原理：批量处理**

与其高频发送小消息，不如积累数据后以较低的频率发送批量消息。

```cpp
// 不佳：每收到一个数据点就发布一次
void sensor_callback(const SensorData& data) {
    auto msg = std::make_unique<std_msgs::msg::Float64>();
    msg->data = data.value;
    publisher_->publish(std::move(msg));
}

// 更优：积累后批量发布
void sensor_callback(const SensorData& data) {
    buffer_.push_back(data.value);
    if (buffer_.size() >= BATCH_SIZE) {
        auto msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
        msg->data = std::move(buffer_);  // 移动语义，避免拷贝
        publisher_->publish(std::move(msg));
        buffer_.clear();
    }
}
```

**效果**：减少了序列化调用次数和上下文切换，提高了整体吞吐量。



#### 通信方式对比总结

| 特性           | Intra-Process              | 共享内存 Inter-Process            | 普通网络传输                   |
| :------------- | :------------------------- | :-------------------------------- | :----------------------------- |
| **通信范围**   | 同一节点内部               | 同一主机上的不同节点              | 跨主机、跨网络                 |
| **性能**       | 极高（零拷贝）             | 高（对大消息近零拷贝）            | 中等（有序列化/网络开销）      |
| **延迟**       | 极低（纳秒级）             | 低（微秒级）                      | 较高（毫秒级）                 |
| **数据拷贝**   | 0-1次                      | 对大消息：0-1次 对小消息：可能2次 | 至少2次（序列化+网络）         |
| **配置复杂度** | 简单                       | 中等                              | 简单（默认）                   |
| **适用数据**   | 所有类型，特别是高频大数据 | 大数据（图像、点云、地图）        | 所有类型，特别是控制命令、状态 |



























