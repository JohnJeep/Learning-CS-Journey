<!--
 * @Author: JohnJeep
 * @Date: 2026-03-09 19:03:37
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-03-09 19:33:08
 * @Description: ROS2 messages, service, action and custom interfaces Usage
 * Copyright (c) 2026 by John Jeep, All Rights Reserved. 
-->

- [1. Introduction](#1-introduction)
- [2. std\_msgs](#2-std_msgs)
- [References](#references)

# 1. Introduction

在 ROS 2 中，消息（Messages）、服务（Services）和动作（Actions）是节点之间通信的基本机制。
ROS 2 提供了丰富的标准消息类型，同时也支持用户定义自定义消息、服务和动作接口，以满足特定应用需求。


查看本地已安装的所有 interfaces
```bash
ros2 interface list
```


# 2. std_msgs

在工程中使用`std_msgs`的典型场景和示例：

1. **导入消息类型**：在编写节点时，首先需要导入所需的`std_msgs`消息类型。
2. **发布消息**：创建一个发布者，用于发布某种`std_msgs`类型的消息。
3. **订阅消息**：创建一个订阅者，接收并处理某种`std_msgs`类型的消息。
4. **自定义消息组合**：在自定义消息中，可以使用`std_msgs`中的基本类型作为组成部分。


标准消息类型
- std_msgs
- sensor_msgs
- geometry_msgs
- nav_msgs
- actionlib_msgs


自定义消息类型



# 3. References

- Interface definition using .msg / .srv / .action files: https://design.ros2.org/articles/legacy_interface_definition.html
- About ROS 2 interfaces: https://docs.ros.org/en/foxy/Concepts/About-ROS-Interfaces.html
- common_interfaces: https://github.com/ros2/common_interfaces
- rcl_interfaces: https://github.com/ros2/rcl_interfaces
- std_msgs: https://docs.ros.org/en/humble/p/std_msgs/
- control_msgs: https://github.com/ros-controls/control_msgs
