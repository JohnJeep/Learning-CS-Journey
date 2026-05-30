<!--
 * @Author: JohnJeep
 * @Date: 2025-11-18 10:57:04
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-03-07 15:23:57
 * @Description: Sensor Usage
 * Copyright (c) 2026 by John Jeep, All Rights Reserved. 
-->



# Sensor

**传感器** 是机器人系统中用于感知环境和自身状态的关键组件。它们将物理世界的信息转换为数字信号，供机器人处理和决策使用。

- **内部传感器（本体感知）：**
  - **编码器：** 测量关节角度和电机转速。
  - **IMU（惯性测量单元）：** 提供机体的加速度、角速度，用于感知姿态和平衡。
  - **扭矩/力传感器：** 测量末端执行器或关节的力和力矩，实现柔顺控制。
  - **电机电流/温度传感器：** 监控电机状态，防止过载。
- **外部传感器（外部感知）：**
  - **摄像头（2D/3D）：** 获取视觉信息，用于物体识别、定位、导航。
  - **激光雷达（LiDAR）：** 生成环境的3D点云地图，用于SLAM（同步定位与建图）和避障。
  - **毫米波雷达：** 在恶劣天气下稳定测距测速。
  - **超声波传感器：** 短距离测距，常用于防撞。
  - **麦克风阵列：** 用于语音交互和声源定位。
  - 深度相机、3D相机、鱼眼相机、双目RGB相机
  - 灵巧手：指尖搭载传感器
  - 舵机

**交互方式：** 这些传感器数据通常通过高速、实时的总线（如 **EtherCAT, CAN bus, SPI, I²C**）以固定的频率发布到机器人内部网络中。






# References

- humble sensor_msgs: https://docs.ros.org/en/humble/p/sensor_msgs/
- humble std_msgs: https://docs.ros.org/en/humble/p/std_msgs/
