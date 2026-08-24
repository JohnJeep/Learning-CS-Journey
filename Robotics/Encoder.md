<!--
 * @Author: JohnJeep
 * @Date: 2026-08-24 08:26:27
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-08-24 09:04:49
 * @Description: Encoder usage
 * Copyright (c) 2026 by John Jeep, All Rights Reserved.
-->

# 1. Introduction

编码器主要用于检测电机或关节的**位置、速度和方向**，是机器人实现精确运动控制的关键传感器。常见的类型有：

- **Incremental Encoder**（增量式编码器）
- **Absolute Encoder**（绝对式编码器）

编码器本质上是一个**测量旋转或直线位移的传感器**，它的核心作用就是告诉控制系统"动了多少、动得多快、往哪动"。具体应用非常广泛：

**🤖 机器人领域**

- **关节位置检测**：实时反馈机械臂每个关节的精确角度，确保动作到位。
- **伺服电机控制**：构成闭环控制，让电机精准运转，不丢步。
- **移动底盘测速**：通过测量轮子的转速，计算机器人的移动速度和行驶距离（里程计）。



 **🏭 工业自动化**

- **数控机床（CNC）**：精确控制刀具的进给位置和速度，保证加工精度。

- **传送带控制**：测量传送带的运行速度和位置，实现精准的物料分拣和定位。

- **电梯控制**：检测电梯轿厢的位置和运行速度，实现精准平层。



# 2. References

- https://doc.embedfire.com/motor/motor_tutorial/zh/latest/index.html
