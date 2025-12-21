<!--
 * @Author: JohnJeep
 * @Date: 2025-12-07 23:39:49
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-12-21 13:40:28
 * @Description: BehaviorTree Usage
 * Copyright (c) 2025 by John Jeep, All Rights Reserved. 
-->

- [1. BehaviorTree](#1-behaviortree)
  - [1.1. 定义](#11-定义)
  - [1.2. 特点](#12-特点)
  - [1.3. 优点](#13-优点)
  - [1.4. 行为树与有限状机对比](#14-行为树与有限状机对比)
- [2. Basical Concepts](#2-basical-concepts)
  - [2.1. Tick](#21-tick)
    - [2.1.1. tick 定义](#211-tick-定义)
    - [2.1.2. tick 的作用](#212-tick-的作用)
    - [2.1.3. tick返回值](#213-tick返回值)
    - [2.1.4. tick 的流程](#214-tick-的流程)
    - [2.1.5. Tick 的特性](#215-tick-的特性)
      - [2.1.5.1. 可中断性（Interruptibility）](#2151-可中断性interruptibility)
      - [2.1.5.2. 状态保持](#2152-状态保持)
      - [2.1.5.3. 响应式更新](#2153-响应式更新)
  - [2.2. Port](#22-port)
    - [2.2.1. 定义](#221-定义)
    - [2.2.2. 特点](#222-特点)
    - [2.2.3. 常用数据类型](#223-常用数据类型)
    - [2.2.4. Port 的底层实现](#224-port-的底层实现)
    - [2.2.5. Port direction](#225-port-direction)
      - [2.2.5.1. Input port(输入端口)](#2251-input-port输入端口)
      - [2.2.5.2. Output port(输出端口)](#2252-output-port输出端口)
      - [2.2.5.3. BidirectionalPort(双向端口)](#2253-bidirectionalport双向端口)
    - [2.2.6. Port 在 XML 中的使用](#226-port-在-xml-中的使用)
    - [2.2.7. Port 高级特性](#227-port-高级特性)
    - [2.2.8. 与网络端口的对比](#228-与网络端口的对比)
  - [2.3. Blackboard](#23-blackboard)
    - [2.3.1. 定义](#231-定义)
    - [2.3.2. 特点](#232-特点)
    - [2.3.3. 用法](#233-用法)
    - [2.3.4. port 与 entry 的关系](#234-port-与-entry-的关系)
    - [2.3.5. 底层绑定机制简述](#235-底层绑定机制简述)
- [3. Nodes](#3-nodes)
  - [3.1. Types of nodes](#31-types-of-nodes)
  - [3.2. controlNode](#32-controlnode)
    - [3.2.1. Sequence](#321-sequence)
    - [3.2.2. Fallback](#322-fallback)
    - [3.2.3. Parallel](#323-parallel)
  - [3.3. LeafNode](#33-leafnode)
    - [3.3.1. ActionNode](#331-actionnode)
    - [3.3.2. ConditionNode](#332-conditionnode)
  - [3.4. DecoratorNode](#34-decoratornode)
  - [3.5. Custom nodes](#35-custom-nodes)
- [4. XML schema](#4-xml-schema)
  - [4.1. XML 的作用](#41-xml-的作用)
  - [4.2. 内置标签](#42-内置标签)
    - [4.2.1. 根标签](#421-根标签)
    - [4.2.2. 行为树定义标签](#422-行为树定义标签)
    - [4.2.3. 控制节点（Control Nodes）](#423-控制节点control-nodes)
    - [4.2.4. 装饰器节点（Decorator Nodes）](#424-装饰器节点decorator-nodes)
    - [4.2.5. 动作节点（Action Nodes）与条件节点（Condition Nodes）](#425-动作节点action-nodes与条件节点condition-nodes)
    - [4.2.6. 其他特殊标签](#426-其他特殊标签)
    - [4.2.7. Ports 与 Blackboard](#427-ports-与-blackboard)
- [5. BehaviorTree.CPP API](#5-behaviortreecpp-api)
  - [5.1. ActionNode](#51-actionnode)
- [6. ROS 中如何实现行为树？](#6-ros-中如何实现行为树)
- [7. Groot](#7-groot)
  - [7.1. 为什么这样设计？](#71-为什么这样设计)
- [8. References](#8-references)


## 1. BehaviorTree

### 1.1. 定义

**行为树** 是一种用于建模AI决策逻辑的**图形化编程架构**。它通过**节点**和**树状结构**来控制任务流程，广泛应用于游戏AI、机器人、自动驾驶等领域，以实现复杂、响应式且易于调试的智能行。

行为树你可以把它想象成一个流程图或决策树，用于控制一个机器人（或游戏中的角色）应该如何根据周围环境的变化来执行任务。

可将行为树中的 Node，想象为 乐高积木 中的一个小模块。



### 1.2. 特点

行为树是树状的结构，它的逻辑流程是由`xml`文件描述的。

行为树本身并不具体实现机器人的执行内容，它只负责将执行内容进行编排。

可用 Groot 工具来可视化行为树。



### 1.3. 优点

1. 调试方便，机器行为的变化都可以追溯。机器行为的变化可记录回放。行为变化也可实时监控。
2. 代码复用率高。不同的功能只需少量代码的修改和机器行为的重新组织。



### 1.4. 行为树与有限状机对比

| 特性       | **有限状态机**(FSM)                                      | **行为树**                                                   |
| :--------- | :------------------------------------------------------- | :----------------------------------------------------------- |
| **结构**   | 网状。状态之间互相跳转，关系复杂。                       | **树状**。层次清晰，从根到叶单向流动。                       |
| **扩展性** | 添加新状态时，需修改多个现有状态间的跳转逻辑，容易出错。 | **模块化强**。添加新分支/节点通常不影响现有逻辑。            |
| **可读性** | 状态多时，“意大利面条”式的跳转线难以理解。               | **直观**。树形结构一目了然，逻辑易于理解和沟通。             |
| **复用性** | 状态逻辑通常绑定具体任务，复用性差。                     | **节点可高度复用**。一个“移动到某点”节点可被多处调用。       |
| **反应性** | 需要在每个状态中检查外部事件，实现较笨拙。               | **天生反应式**。每次“滴答”都从根节点重新评估，能快速响应变化。 |

**行为树相比状态机的优势**：

- 模块化与可复用性： 节点（如“导航到A点”）是独立的，可以在树的不同位置重复使用。

- 可维护性： 树形结构非常直观，易于理解、调试和修改。添加新行为就像在树上添加一个新分支。

- 动态性： 行为树可以“反应式”地响应环境变化。例如，当一个机器人正在执行“前往目标点”的任务时，如果电池电量低，它可以立即中断当前任务，转而执行“返回充电”任务。
- 可伸缩性： 可以轻松地构建非常庞大和复杂的行为逻辑，而不会变得像“意大利面条”一样的代码。

**状态机弊端**：

状态与执行内容是绑定在一起的。当执行内容需要在多个状态中执行时，各个状态下都需要放置执行内容的逻辑。当业务逻辑代码分散在各处时就不太好维护了，特别是对于复杂的机器人系统。



## 2. Basical Concepts

### 2.1. Tick

#### 2.1.1. tick 定义

**tick 是一个 signal**。tick signal 发送到树的根节点，并通过树传播直到到达叶节点。**tick** 是行为树的一次更新调用。你可以把它想象成一次“心跳”或“脉冲”，每次 tick 都会从根节点开始，按照一定的规则遍历树，更新节点的状态，并最终决定 AI 当前应该做什么。

#### 2.1.2. tick 的作用

1. **驱动行为树执行**：每次 tick 都会让行为树从根节点开始执行，根据节点的逻辑和状态，决定哪些节点应该被运行、哪些应该被跳过。
2. **状态更新**：在 tick 过程中，节点会根据当前游戏世界的状态（通过黑板或上下文）来更新自己的状态，并返回运行结果（成功、失败、运行中）。
3. **实现持续行为**：对于需要多帧完成的行为（如移动、等待），节点可以在一次 tick 中返回“运行中”（RUNNING），这样在下一次 tick 时，行为树会从上次中断的地方继续执行（对于有状态的节点）或重新开始（对于无状态的节点）。

#### 2.1.3. tick返回值

任何接收到 tick 信号的 TreeNode 都会执行其回调函数(callback)。回调返回的结果如下

- 成功(SUCCESS)： 节点已完成其任务。

- 失败(FAILURE)： 节点未能完成其任务。

- 运行中(RUNNING)： 节点正在执行中，尚未完成（例如，机器人正在移动中）。这是实现“反应性”的关键。

#### 2.1.4. tick 的流程

当行为树被 tick 时，它会从根节点开始，按照节点类型定义的逻辑来遍历子节点。例如：

- **控制流节点**（如序列、选择、并行）会按照特定顺序 tick 其子节点，并根据子节点的返回值决定下一步。
- **条件节点**（Condition）会检查某个条件，立即返回成功或失败。
- **动作节点**（Action）会执行一个动作，可能立即完成（返回成功/失败）或持续多帧（返回运行中）。



#### 2.1.5. Tick 的特性

##### 2.1.5.1. 可中断性（Interruptibility）

```cpp
// 假设有以下行为树：
// <Sequence>
//     <Wait duration="5"/>
//     <PlayAnimation name="dance"/>
// </Sequence>

// Tick 1: Wait -> RUNNING（已等待0.5秒）
// Tick 2: Wait -> RUNNING（已等待1.0秒）
// Tick 3: 玩家按了取消键 -> 行为树被打断
// Tick 4: 执行取消逻辑，不再继续等待
```

##### 2.1.5.2. 状态保持

```cpp
class ChargingAttack : public BT::StatefulActionNode {
    float charge_time = 0;
    NodeStatus onRunning() override {
        charge_time += getDeltaTime();
        if (charge_time >= 3.0f) {
            releaseAttack();  // 释放攻击
            return NodeStatus::SUCCESS;
        }
        continueCharging();   // 继续蓄力
        return NodeStatus::RUNNING;  // 告诉行为树："我还没完成"
    }
};
```

##### 2.1.5.3. 响应式更新

```cpp
// 每帧都可以根据新情况做决策
void tickEnemyAI() {
    // Tick 1: 玩家在远处 -> 选择"巡逻"分支
    // Tick 2: 玩家进入视野 -> 切换到"追击"分支
    // Tick 3: 玩家逃跑 -> 切换到"返回巡逻"分支
    // Tick 4: 玩家进入攻击范围 -> 切换到"攻击"分支
}
```

### 2.2. Port

理解 Port 机制是掌握 BehaviorTree 高级用法的关键，它让行为树从简单的状态机变成了真正的数据流驱动的决策系统。

#### 2.2.1. 定义

- 定义：Port 是一种 nodes 之间可以彼此交换信息的机制。Port 允许你在行为树节点（如 ActionNode、ConditionNode、ControlNode 等）之间传递数据，而无需硬编码依赖。

- > **Port 是行为树节点与外部数据（尤其是 Blackboard）之间的标准化、类型安全的接口抽象，使得行为树具备高度的灵活性、可配置性和复用性。**

  它不是传统意义上的“网络端口”或“硬件端口”，而是一种 **行为树内部的数据绑定契约机制**。提供了一种简介操作，也叫 `remapping`。

- 作用：让节点能够**接收输入参数**和**输出执行结果**。

#### 2.2.2. 特点

1. **参数化**：使节点可配置，提高复用性
2. **数据流**：在节点间传递数据，实现复杂逻辑
3. **解耦**：节点不直接依赖具体数据，通过端口交互
4. **灵活性**：支持运行时参数绑定和修改
5. **支持静态值与动态Blackboard 引用**
    Port 可以绑定：
   - **静态值**（如 `"42"`、`"true"`、`"{x:1, y:2}"`）
   - 动态**Blackboard key**：运行时从共享的 Blackboard 中获取/设置。

#### 2.2.3. 常用数据类型

数据类型可以是任意 c++类型。

- `double` - 浮点数
- `int` - 整数
- `bool` - 布尔值
- `std::string` - 字符串
- `Pose2D` - 位姿（格式：`x;y;theta`）

#### 2.2.4. Port 的底层实现

- **类型**：在 BehaviorTree.CPP 中，所有 Port 的值在内部以 `BT::StringView` 或 `std::string` 形式存储（序列化为字符串），但在使用时会通过模板自动转换为目标类型（如 `int`、`Pose2D` 等）。
- **注册机制**：在定义自定义节点时，通过 `providedPorts()` 静态方法声明该节点支持哪些输入/输出端口及其类型。
- **与 Blackboard 绑定**：当 XML 中将 Port 值写成大括号形式（如 `{target}`），库会将其解释为对 Blackboard 中 `target` 键的引用。

#### 2.2.5. Port direction

port 以下几种类型。

| 类型                             | 说明                            |
| -------------------------------- | ------------------------------- |
| `InputPort<T>`                   | 只读，用于接收外部数据          |
| `OutputPort<T>`                  | 只写，用于输出结果到 Blackboard |
| `BidirectionalPort<T>`（较少用） | 可读可写（需谨慎使用）          |

##### 2.2.5.1. Input port(输入端口)

```cpp
static BT::PortsList providedPorts() {
    return {
        // 基本类型
        BT::InputPort<int>("count"),
        BT::InputPort<double>("distance"),
        BT::InputPort<std::string>("target_name"),

        // 带默认值
        BT::InputPort<int>("timeout", 5000),  // 默认值 5000ms

        // 复杂类型
        BT::InputPort<Pose>("target_pose"),
        BT::InputPort<std::vector<Point>>("path_points")
    };
}
```

##### 2.2.5.2. Output port(输出端口)

```cpp
static BT::PortsList providedPorts() {
    return {
        BT::OutputPort<bool>("detected"),      // 输出检测结果
        BT::OutputPort<float>("confidence"),   // 输出置信度
        BT::OutputPort<Pose>("current_pose")   // 输出当前位置
    };
}
```

当你在 XML 中写：

```xml
<JsonToolGet output="{result}" />
```

BehaviorTree.CPP 在解析时会：

1. 发现 `output` 是一个 **输出端口（output port）**
2. 将 `{result}` 解析为 Blackboard 键名 `result`
3. 当你的 C++ 节点调用 `setOutput("value", 11)` 时
4. 框架自动执行 `blackboard->set("result", 11)`

> 🔧 **底层等价于**：
>
> ```cpp
> // 你在 tick() 中写的
> setOutput("output", 11);
> 
> // 框架实际执行的
> blackboard->set("result", 11);  // 因为 XML 中 output="{result}"
> ```



##### 2.2.5.3. BidirectionalPort(双向端口)

```cpp
static BT::PortsList providedPorts() {
    return {
        BT::BidirectionalPort<std::string>("message")  // 可读写
    };
}
```

#### 2.2.6. Port 在 XML 中的使用

参数传递示例

```xml
<root>
    <BehaviorTree>
        <!-- 通过端口传递参数 -->
        <Sequence>
            <!-- 设置目标点 -->
            <SetTarget 
                target_x="10.0" 
                target_y="5.0" 
                target_name="桌子"/>
            
            <!-- 导航到目标，使用前一个节点的输出 -->
            <NavigateTo 
                target="{target_pose}" 
                timeout_ms="30000"/>
            
            <!-- 抓取物体 -->
            <GraspObject 
                object_name="{target_name}" 
                force="0.5"/>
        </Sequence>
    </BehaviorTree>
</root>
```

数据流示例

```xml
<root>
    <BehaviorTree>
        <Sequence>
            <!-- 节点A：输出数据 -->
            <DetectObject 
                object_id="cup"
                output_detected="{cup_detected}"
                output_position="{cup_position}"/>
            
            <!-- 节点B：使用节点A的输出作为输入 -->
            <MoveToObject 
                target_position="{cup_position}"
                only_if="{cup_detected}"/>
            
            <!-- 节点C：输出新数据 -->
            <GraspObject 
                object_position="{cup_position}"
                output_grasp_success="{grasp_ok}"/>
            
            <!-- 节点D：使用多个端口数据 -->
            <ReportStatus 
                object_name="cup"
                detected="{cup_detected}"
                grasp_result="{grasp_ok}"/>
        </Sequence>
    </BehaviorTree>
</root>
```



#### 2.2.7. Port 高级特性

1. 动态端口

   ```cpp
   static BT::PortsList providedPorts() {
       // 动态端口：运行时决定
       BT::PortsList ports;
       ports.insert(BT::InputPort<std::string>("dynamic_input"));
       ports.insert(BT::OutputPort<int>("dynamic_output"));
       return ports;
   }
   ```

2. Blackboard 机制

   ```cpp
   // Blackboard 是全局共享的数据存储
   BT::Blackboard::Ptr blackboard = BT::Blackboard::create();
   
   // 设置全局参数
   blackboard->set("global_speed", 0.5);
   blackboard->set("emergency_stop", false);
   
   // 在节点中访问
   BT::NodeStatus MyNode::tick() override {
       double speed;
       if (getInput("speed", speed)) {
           // 使用输入端口的值
       } else {
           // 使用黑板的默认值
           config().blackboard->get("global_speed", speed);
       }
       // ...
   }
   ```

3. 端口验证和转换

   ```cpp
   // 自定义端口验证器
   BT::PortsList providedPorts() {
       return {
           BT::InputPort<int>("value", "必须为正数", [](BT::PortInfo& info){
               auto value = info.getValue<int>();
               return value > 0;  // 验证输入是否为正数
           }),
           
           BT::InputPort<std::string>("filename", "必须是文本文件", [](BT::PortInfo& info){
               auto filename = info.getValue<std::string>();
               return filename.ends_with(".txt");
           })
       };
   }
   ```

#### 2.2.8. 与网络端口的对比

| 维度     | **BehaviorTree Port** | **网络端口**    |
| :------- | :-------------------- | :-------------- |
| **本质** | 数据接口和参数通道    | 网络通信端点    |
| **用途** | 节点间数据传递        | 网络连接和通信  |
| **类型** | 输入、输出、双向      | TCP、UDP 等     |
| **数据** | 任意 C++ 类型         | 字节流          |
| **范围** | 单个行为树内部        | 网络间通信      |
| **连接** | 静态/动态绑定         | 网络连接        |
| **协议** | 无协议，直接访问      | TCP/IP、HTTP 等 |



### 2.3. Blackboard

#### 2.3.1. 定义

在 BT.CPP 中，Blackboard 是一个（key/value）键值对，用于在节点之间共享数据。

每个节点可以有一些 input port 和 output port，这些 port 可以与 Blackboard 上的 entry 进行绑定。

#### 2.3.2. 特点

1. 树中所有节点都共享键值对。
2. **entry**：是 Blackboard  的 **key/value pair**。它是运行时的共享数据，用户层写代码时，不直接显示调用。
3. **Input port**：可以 read  entry；**Out por**t：可以写数据到 entry。所有节点都可以从 blackboard **读取**数据（如“敌人位置”、“剩余电量”）或向 blackboard **写入**数据（如“设置目标点=”）。
4. **作用**：实现节点间的数据通信，解耦逻辑与数据。XML 中声明绑定关系，代码中运行时解析，动态建立 port-Blackboard 连接。

#### 2.3.3. 用法

在 **BehaviorTree.CPP** 的 XML 定义中，`"{blackboard_key}"` 是将 **Port（端口）与 Blackboard 条目（Entry）绑定的标准方式**。

其XML 中 Port 与 Blackboard 绑定的通用格式如下：

```xml
<NodeName port_name="{blackboard_key}" />
```

- `NodeName`：行为树节点类型（如 `MoveTo`, `Condition`, `Fallback` 等）。
- `port_name`：该节点在 C++ 中通过 `providedPorts()` 声明的 **Port 名称**。
- `"{blackboard_key}"`：
  - **必须用双引号包裹整个值**。
  - **花括号 `{}` 表示这是一个 Blackboard 引用**。若不加 {}，仅仅表示普通的字符串。
  - **花括号内的内容就是 Blackboard 的键（key）**。

在 XML 文件使用 `{}` 与 InputPort 和 OutPort 进行绑定 。 当一个节点设置 `port_name="{blackboard_key}"`时，它会将结果存储在 Blackboard  的`blackboard_key`上；其他节点通过 `input="{blackboard_key}"` 可以获取这个值。

**示例**：

1. 自定义节点声明（C++）

   ```cpp
   class MoveTo : public BT::SyncActionNode
   {
   public:
       MoveTo(const std::string& name, const BT::NodeConfig& config)
           : BT::SyncActionNode(name, config) {}
   
       static BT::PortsList providedPorts()
       {
           return {
               BT::InputPort<Position>("target"),      // 输入端口
               BT::OutputPort<int>("steps_moved")     // 输出端口
           };
       }
   
       BT::NodeStatus tick() override { /* ... */ }
   };
   ```

2. XML 中绑定 Port 到 Blackboard

   ```xml
   <root main_tree_to_execute="MainTree">
     <BehaviorTree ID="MainTree">
       <Sequence>
         <!-- 绑定 InputPort "target" 到 Blackboard 的 "goal" -->
         <!-- 绑定 OutputPort "steps_moved" 到 Blackboard 的 "total_steps" -->
         <MoveTo target="{goal}" steps_moved="{total_steps}"/>
       </Sequence>
     </BehaviorTree>
   </root>
   ```

3. 运行时：

   - `target` 的值 = `blackboard.get<Position>("goal")`
   - 执行后：`blackboard.set<int>("total_steps", steps_value)`

#### 2.3.4. port 与 entry 的关系

1. Port：是 node 定义的一部分，用于指定 node 需要输入的数据或将要输出的数据。在节点类中，通过`providedPorts()`静态方法来声明 port，并在`tick()`函数中使用`getInput()`读取输入 port 或使用`setOutput()`写入输出 port。

2. Entry：是 blackboard 上存储数据的键值对。每个 entry 有一个键（key）和一个值（value）。在XML中，使用花括号`{key}`来引用 blackboard  上的 条目（entry）。

3. **绑定关系**：在行为树XML中，将 node 的 port 与 entry 进行绑定。例如：

   ```xml
   <SaySomething message="{target}" />
   ```

   表示将 SaySomething 节点的输入端口"message"与 blackboard  上的 entry "target" 绑定。当节点执行时，它会从 blackboard  上的 entry  "target"中读取值作为输入。

   ```xml
   <ThinkWhatToSay text="{target}" />
   ```

   表示将 ThinkWhatToSay 节点的输出端口 "text"与 blackboard  上的 entry  "target" 绑定。当节点执行时，它输出的值会写入 blackboard  上的 entry  "target"。

**关键区别**

| 特性         | Blackboard Entry | Node Port            |
| :----------- | :--------------- | :------------------- |
| **作用域**   | 全局（整个树）   | 局部（单个节点）     |
| **生命周期** | 与树同生命周期   | 与节点调用同生命周期 |
| **数量限制** | 无限制           | 在节点类中固定定义   |
| **访问方式** | 通过键名访问     | 通过端口名访问       |
| **数据流向** | 存储数据         | 传输数据             |

在节点代码中，使用`getInput`来读取输入 port，使用`setOutput`来写入输出 port。这些函数内部会自动处理与 blackboard 的交互（如果 port 绑定到了entry）。如果port 没有绑定到 entry，而是直接给定了一个固定值，那么`getInput`会直接返回那个固定值。

#### 注意点

**推荐使用 Port 去操作数据而不是直接使用 Blackboard 去操作数据。**

推荐写：

```cpp
// example code in your tick()
getInput("goal", goal);
setOutput("result", result);
```

避免写：

```cpp
// example code in your tick()
config().blackboard->get("goal", goal);
config().blackboard->set("result", result);
```

#### 2.3.5. 底层绑定机制简述

当 BehaviorTree.CPP 加载 XML 时：

1. 对每个节点属性（即 Port 赋值），检查值是否匹配正则：`^\{(.+)\}$`
2. 如果匹配：
   - 提取内部字符串作为 **Blackboard key**
   - 在节点初始化时，将该 Port 标记为“动态绑定”
3. 运行时（tick()）前后）：
   - **InputPort**：自动从 Blackboard 读取 `key` 对应的值（类型安全）
   - **OutputPort**：自动将结果写回 Blackboard 的 `key`

这个过程对用户透明，无需手动调用 `get()`/`set()`。



## 3. Nodes

任何 TreeNode 都可以看作是一种调用回调函数（callback）的机制，也就是执行一段代码。这个回调函数具体做什么，由使用者自己决定。

### 3.1. Types of nodes

1. ControlNode
2. DecoratorNode
3. LeafNode
   1. ActionNode：要区分 synchronous, asynchronous nodes
   2. ConditionNode

| Type of TreeNode | Children Count | Notes                                                        |
| ---------------- | -------------- | ------------------------------------------------------------ |
| ControlNode      | 1...N          | Usually, ticks a child based on the result of its siblings or/and its own state. |
| DecoratorNode    | 1              | Among other things, it may alter the result of its child or tick it multiple times. |
| ConditionNode    | 0              | Should not alter the system. Shall not return RUNNING.       |
| ActionNode       | 0              | This is the Node that "does something"                       |

### 3.2. controlNode

controlNode 叫控制节点。像树干和树枝，负责控制子节点的执行流程。

这是行为树中的控制流。类似`c++`语言中的`if else`，`switch`等等。它负责构建行为树的逻辑结构。`sequeence`，`fallback`等等就属于这个范畴。

#### 3.2.1. Sequence

Sequence 叫序列节点。最基本和最常用。

- **逻辑**：按顺序执行所有子节点。**只有全部成功，它才返回成功；任何一个失败，则立即停止并返回失败。**
- **例子**：`序列[接近门，开门，通过，关门]` —— 任何一个步骤失败（如门打不开），整个任务失败。

提供三种类型 nodes

- Sequence
- SequenceWithMemory
- ReactiveSequence：响应式序列。

#### 3.2.2. Fallback

FallbackNode 叫选择节点或者选择器(Selector)。

- **逻辑**：按顺序执行子节点，**直到有一个成功为止**。即“尝试方案A，不行就试B，还不行就试C...”。
- **例子**：`选择[用钥匙开门， 用力拉门， 呼叫管理员]` —— 尝试各种开门方法，直到一个成功。

![](./figures/sequences_fallbacks.png)

提供2中 nodes l类型

- Fallback
- ReactiveFallback



#### 3.2.3. Parallel

parallel 叫并行节点。

- **逻辑**：同时执行所有子节点。可根据成功/失败数量阈值来决定自身返回结果。
- **例子**：`并行[播放音乐， 闪烁灯光]` —— 同时完成多个动作，营造氛围。



### 3.3. LeafNode

LeafNode 叫执行节点或者叶子节点。在树的最末端，像树叶，是实际“干活”的节点。它没有任何的 children，是实际的 命令。

#### 3.3.1. ActionNode

ActionNode 叫动作节点，**最常用的类型**。

- 动作节点通常实现服务客户端和动作客户端，也可以是一些简单的执行程序。他们通过向`Planner server`，`Controller server`，`Recovery server`发送请求来启动相应的功能程序。`action`通常作为行为树中的叶子节点，负责具体行为和功能的实现。但这些具体的功能代码并没有在叶子节点中而是在对应的服务端。

- **做什么**：执行一个具体的动作或任务（如“移动”、“抓取”、“说话”）。
- **特点**：可能耗时，会返回“**运行中**”，直到动作完成或失败。

#### 3.3.2. ConditionNode

ConditionNode 叫条件节点。

- 条件控制节点。比如判断电池电量，某一开关信号等等。
- **检查什么**：检查一个世界状态是否成立（如“生命值<30%？”、“看到敌人了吗？”）。
- **特点**：瞬间完成，只返回**成功**或**失败**，不改变世界状态。

### 3.4. DecoratorNode

decoratorNode 叫装饰节点。像树上的装饰，用于修饰或调整单个子节点的行为。比如将子节点的结果进行反向，约束子节点的执行次数等等。

**它只能有一个子节点。**

**常用类型**：

- invert：将子节点的结果反转（SUCCESS 变 FAILURE ，FAILURE 变SUCCESS ，若一个 child 返回 RUNNING，node 也返回 RUNNING）。
- Repeat：反复执行子节点 N 次。
  - child 执行 N 次，若尝试 N 次都返回 SUCCESS ，则 node 总是返回 SUCCESS 。
  - 若 child 返回 FAILURE  ，则中断循环，node 返回 FAILURE 。

- RetryUntilSuccessful：
  - child 执行 N 次，若尝试 N 次都返回 FAILURE ，则 node 总是返回 FAILURE 。
  - 若 child 返回 SUCCESS ，则中断循环，node 返回 SUCCESS。

- KeepRunningUntilFailure：反复执行子节点直到其返回失败。
- Force Success/Failure：无论子节点结果如何，都返回指定状态（失败或成功）。
- Delay：指定多久去 tick child。
- RunOnce：只执行 child 一次。



### 3.5. Custom nodes

通常自定义一个 TreeNode，继承自 `TreeNode`类，或者更具体地说，继承其派生类：

- `ActionNodeBase`
- `ConditionNode`
- `DecoratorNode`



## 4. XML schema

`BehaviorTree.CPP` 是一个用 C++ 编写的强大且灵活的行为树（Behavior Tree）库，广泛用于机器人、游戏 AI 等领域。它支持通过 XML 文件来定义行为树的结构和逻辑，使得行为树可以在运行时动态加载和修改。

在 `BehaviorTree.CPP` 中，XML 文件主要用于描述**行为树的结构**，而不是具体节点的实现逻辑（节点的实现仍需在 C++ 代码中注册）。

### 4.1. XML 的作用

- 定义行为树的结构（哪些节点、怎么组合）。
- 指定每个节点的 port 要从 blackboard 的哪个 key 读取或写入。

### 4.2. 内置标签

XML 的标签遵循一定的规则，**XML 属性值必须加引号**。

通常 XML 中的标签名可以任意写。但 BehaviorTree 中内置节点对应的 XML 标签名，**内置的标准标签**大致如下：

| 类别           | 数量（约）                                        |
| -------------- | ------------------------------------------------- |
| 控制节点       | 7–9 个                                            |
| 装饰器节点     | 10 个左右                                         |
| 特殊结构标签   | 2–3 个（`<root>`, `<BehaviorTree>`, `<SubTree>`） |
| 用户自定义节点 | 任意数量（由注册决定）                            |

因此，**标准内置标签大约 20 个左右**，加上用户自定义的叶子节点，总数是可扩展的。



主要分为以下几类：

#### 4.2.1. 根标签

`<root>`：XML 文件的根元素。

可选属性：

- `main_tree_to_execute`：指定要执行的主行为树名称（当 XML 中定义了多个树时使用）。

  1. **主树入口**：当 XML 文件中包含多个行为树时，`main_tree_to_execute` 指定了 **程序启动时默认执行哪一个树**。XML 中只有一个行为树时，可以不用写。
  2. **树之间的调用**：BehaviorTree.CPP 支持树之间的相互调用（通过 `SubTree` 或 `SubTreePlus` 节点），因此一个 XML 文件可以包含多个独立的树结构。

  ```xml
  <root BTCPP_format="4" main_tree_to_execute="MainTree">
    <!-- 主树定义 -->
    <BehaviorTree ID="MainTree">
      <Sequence>
        <Action ID="Action1"/>
        <SubTree ID="SubTree1"/>
      </Sequence>
    </BehaviorTree>
    
    <!-- 子树定义 -->
    <BehaviorTree ID="SubTree1">
      <Sequence>
        <Condition ID="Condition1"/>
        <Action ID="Action2"/>
      </Sequence>
    </BehaviorTree>
  </root>
  ```

- `BTCPP_format`：Behavior Tree 的版本。

#### 4.2.2. 行为树定义标签

`<BehaviorTree>`：定义一棵行为树。

- 必须有 `ID` 属性，作为该树的唯一标识。

- 示例：

  ```xml
  <BehaviorTree ID="MainTree">
    <!-- 节点内容 -->
  </BehaviorTree>
  ```

#### 4.2.3. 控制节点（Control Nodes）

这些节点用于控制子节点的执行流程：

| 标签名               | 功能说明                                                     |
| -------------------- | ------------------------------------------------------------ |
| `<Sequence>`         | 顺序节点：依次执行子节点，一旦某个子节点返回 FAILURE，则停止并返回 FAILURE；全部成功才返回 SUCCESS。 |
| `<SequenceStar>`     | 带记忆的 Sequence（状态持久化），用于中断恢复。              |
| `<Fallback>`         | 选择节点（Selector）：依次执行子节点，一旦某个子节点返回 SUCCESS，则停止并返回 SUCCESS；全部失败才返回 FAILURE。 |
| `<FallbackStar>`     | 带记忆的 Fallback。                                          |
| `<Parallel>`         | 并行节点：同时执行所有子节点，根据成功/失败阈值决定返回值。可带属性 `success_threshold` 和 `failure_threshold`。 |
| `<ReactiveSequence>` | 非记忆型 Sequence，每 tick 从头开始评估。                    |
| `<ReactiveFallback>` | 非记忆型 Fallback。                                          |

> 注意：旧版本中可能使用 `<Selector>`，但在 v3+ 中推荐使用 `<Fallback>`。

#### 4.2.4. 装饰器节点（Decorator Nodes）

用于包装单个子节点，改变其行为：

| 标签名                      | 功能说明                                       |
| --------------------------- | ---------------------------------------------- |
| `<Inverter>`                | 反转子节点结果（SUCCESS ↔ FAILURE）。          |
| `<RetryUntilSuccessful>`    | 重复执行子节点直到成功（可指定次数）。         |
| `<KeepRunningUntilFailure>` | 子节点返回 RUNNING 时继续，FAILURE 时停止。    |
| `<Repeat>`                  | 重复执行 N 次（属性 `num_cycles`）。           |
| `<Timeout>`                 | 设置超时（属性 `msec`），超时则返回 FAILURE。  |
| `<ForceSuccess>`            | 强制子节点返回 SUCCESS。                       |
| `<ForceFailure>`            | 强制子节点返回 FAILURE。                       |
| `<WhileSuccess>`            | 只要子节点返回 SUCCESS 就重复执行。            |
| `<IgnoreFailure>`           | 将 FAILURE 转为 SUCCESS。                      |
| `<RunOnce>`                 | 第一次 tick 执行子节点，之后直接返回 SUCCESS。 |

#### 4.2.5. 动作节点（Action Nodes）与条件节点（Condition Nodes）

这两类都属于 **叶子节点（Leaf Nodes）**，在 XML 中以自定义标签名出现，但必须事先在 C++ 代码中注册。

例如：

```xml
<ApproachObject />
<CheckBattery />
```

> 这些标签名必须与你在 `BT::BehaviorTreeFactory` 中注册的名称完全一致。

#### 4.2.6. 其他特殊标签

- `<SubTree>`：引用另一棵已定义的行为树（类似函数调用）。

  - 属性：

    - `ID`：被调用的子树 ID。
    - 可传递端口参数（通过属性或 `<remap>`）。

  - 示例：

    ```xml
    <SubTree ID="CheckAndRecharge" battery_level="{battery}" />
    ```

- `<Remap>`：在 `<SubTree>` 内部用于端口重映射（较新版本中通常直接在属性中完成）。

- `<include>`：引用外部的文件，类似于C++中的 `#include <file>` 的语法。从 2.4 版本开始支持。

  ```xml
  <!-- 格式 -->
  <include path="relative_or_absolute_path_to_file">
  ```

  示例：

  ```xml
   <!-- file maintree.xml -->
  
   <root BTCPP_format="4" >
  	 
  	 <include path="grasp.xml"/>
  	 
       <BehaviorTree ID="MainTree">
          <Sequence>
             <Action  ID="SaySomething"  message="Hello World"/>
             <SubTree ID="GraspObject"/>
          </Sequence>
       </BehaviorTree>
    </root>
  ```

  ```xml
   <!-- file grasp.xml -->
  
   <root BTCPP_format="4" >
       <BehaviorTree ID="GraspObject">
          <Sequence>
             <Action ID="OpenGripper"/>
             <Action ID="ApproachObject"/>
             <Action ID="CloseGripper"/>
          </Sequence>
       </BehaviorTree>  
   </root>
  ```

  ROS中的用法

  ```xml
  <include ros_pkg="name_package"  path="path_relative_to_pkg/grasp.xml"/>
  ```

  

#### 4.2.7. Ports 与 Blackboard

- 节点可通过 `{variable}` 语法访问 Blackboard 上的变量。

- 在 XML 中定义端口，通常通过节点属性完成，例如：

  ```xml
  <NodeName port_name="{blackboard_key}" />
  ```

- 节点的输入/输出端口需在 C++ 注册时声明。



## 5. BehaviorTree.CPP API

关键类

1. **`Blackboard`**: 存储键值对
2. **`TreeNode`**: 基类，包含端口映射
3. **`NodeConfiguration`**: 节点配置，包含端口和黑板引用
4. **`BehaviorTreeFactory`**: 工厂类，解析 XML 并建立绑定
5. **`XMLParser`**: 解析 XML 并创建端口映射



### 5.1. ActionNode

- `StatefulActionNode`：异步接口。是一个特殊类型的动作节点，它在多个tick之间保持状态，适合处理需要持续执行的动作。

  主要用途：

  1. **持续执行的动作**

     - 移动到一个位置（可能需要多个tick）

     - 等待一段时间

     - 执行需要多个帧的动画

     - 与NPC对话交互

  2. **状态保持**

     与普通的 `ActionNode` 不同，`StatefulActionNode` 会记住自己的执行状态，即使在多个tick之间也不会重置。

- `SyncActionNode`：同步接口

- `AsyncActionNode`（不推荐，因为`StatefulActionNode`是更好的替代）。需要手动去管理线程。



## 6. ROS 中如何实现行为树？

在 ROS 中，行为树不是一个官方的包，而是由社区开发和维护的。最流行、最成熟的库是 **BehaviorTree.CPP**。

**BehaviorTree.CPP 库的特点：**

1. **C++库**： 是一个独立的 C++ 库，可以很好地与 ROS 集成。
2. **Groot GUI 工具**： 提供了一个强大的图形化工具，用于**设计、监控和调试**行为树。你可以拖拽节点来构建树，并实时看到节点的状态变化（用颜色表示），这对于开发和调试至关重要。
3. **XML 定义**： 行为树的结构可以用 XML 文件来定义，这使得树的修改变得非常容易，无需重新编译代码。
4. **与 ROS 通信**： 你的“动作”和“条件”节点内部是通过 ROS 的**Actionlib**、**Service**、**Topic** 等机制与机器人系统的其他部分（如导航、感知、控制）进行通信的。



## 7. Groot

使用 Groot 工具查看 xml 文件时，得加 **节点模型**（`<TreeNodesModel>`）后，groot 编辑器才能渲染后现实，若只有 **树结构**（`<BehaviorTree>`）标签，是不能加载的。

### 7.1. 为什么这样设计？

**关注点分离：**

1. **树结构**（`<BehaviorTree>`）：定义行为逻辑
2. **节点模型**（`<TreeNodesModel>`）：定义编辑界面

**优势：**

- **一份代码，多种用途**：同一个XML既用于编辑又用于运行
- **向后兼容**：BehaviorTree.CPP 忽略不认识的 `TreeNodesModel`
- **编辑器友好**：Groot 有足够的信息渲染GUI



## 8. References

1. BehaviorTree.CPP: https://www.behaviortree.dev/
1. **BT::TreeNode Class Reference**:https://docs.ros.org/en/noetic/api/behaviortree_cpp/html/classBT_1_1TreeNode.html
2. 知乎：ROS2中的行为树 BehaviorTree：https://zhuanlan.zhihu.com/p/534072049

