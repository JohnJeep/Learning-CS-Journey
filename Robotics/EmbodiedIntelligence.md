<!--
 * @Author: JohnJeep
 * @Date: 2025-12-28 11:06:27
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-12-29 01:31:52
 * @Description: Embodied Intelligence
 * Copyright (c) 2025 by John Jeep, All Rights Reserved. 
-->

- [1. Embodied Intelligence(具身智能)](#1-embodied-intelligence具身智能)
- [2. Robotic System(机器人系统)](#2-robotic-system机器人系统)
- [3. Forward Kinematics(正向运动学)](#3-forward-kinematics正向运动学)
- [4. Inverse Kinematics(逆向运动学)](#4-inverse-kinematics逆向运动学)
- [5. Forward Dynamics(正向动力学)](#5-forward-dynamics正向动力学)
  - [5.1. 反馈控制](#51-反馈控制)
- [6. Control Synthesis(控制综合)](#6-control-synthesis控制综合)
- [7. ZPM](#7-zpm)


# 1. Embodied Intelligence(具身智能)


线性代数在机器人中的角色。

| 应用领域      | 核心线性代数概念       | 作用                               |
| :------------ | :--------------------- | :--------------------------------- |
| **运动学**    | 齐次变换矩阵、旋转矩阵 | 计算机械臂末端位置、机器人位姿变化 |
| **动力学**    | 质量矩阵、向量运算     | 计算力、力矩、加速度，实现精准控制 |
| **视觉/感知** | 像素矩阵、特征值分解   | 图像识别、三维空间重建、避障       |
| **状态估计**  | 协方差矩阵、矩阵求逆   | 融合传感器数据，消除噪声，准确定位 |

# 2. Robotic System(机器人系统)



# 3. Forward Kinematics(正向运动学)

如何根据机器人的关节角度，描述每个连杆的位置和姿态，被称为正向运动学。



# 4. Inverse Kinematics(逆向运动学)

根据每个连杆的位置和姿态，描述机器人关节角度，被称为逆向运动学。



# 5. Forward Dynamics(正向动力学)

## 5.1. 反馈控制

人形机器人的步态模式要能防止机器人自身在没有干扰的情况下翻到，一般是通过反馈控制来稳定的。

线性倒立摆

- 一维

- 二维

- 三维



生成大致的全身运动模式的方法

1. 动作捕捉
2. GUI
3. 快速高阶空间搜索

> 稳定的执行这些动作。

消除因 GUI和快速高阶空间搜索制作的动作，没有考虑到动力学的特性，而采取的措施：

1. 动态滤波器
2. 自动平衡器



# 6. Control Synthesis(控制综合)





# 7. ZPM

Zero Moment Point（零力矩点）：指机器人脚底从地面承受的力矩为 0 的点，总是在地面和脚底接触面的某个地方。

