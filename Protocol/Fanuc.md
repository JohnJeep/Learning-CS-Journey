<!--
 * @Author: JohnJeep
 * @Date: 2025-04-09 15:34:38
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-04-10 11:12:54
 * @Description: 发那科控制协议
 * Copyright (c) 2025 by John Jeep, All Rights Reserved. 
-->


# 1. Introduction

FANUC（发那科）是工业自动化和机器人领域的知名公司，总部在日本。他们的产品应该涵盖数控系统、工业机器人、自动化设备等。

# 2. Product Overview

1. 数控系统（CNC）

   - **Series 0i**

     经济型CNC，适用于中小型机床（如铣床、车床），操作简便，适合基础加工需求。

   - **Series 30i/31i/32i**

     高端多轴控制系统，支持5轴联动和复杂曲面加工，应用于航空、汽车精密零部件制造。

   - **Power Motion i**

     集成运动控制与CNC功能，专攻高精度伺服控制场景（如电子元件装配）。

2. 工业机器人

   - **M系列（如M-20iA、M-710）**

     高负载（10-700kg）机器人，用于搬运、焊接、喷涂，汽车行业应用广泛。

   - **ARC系列（ARC Mate）**

     专为弧焊设计，高重复定位精度（±0.08mm），适应复杂焊缝轨迹。

   - **LR Mate系列**

     轻型协作机器人（负载5-20kg），紧凑设计，适用于狭窄空间（如3C行业装配）。

   - **SCARA系列（SR-3iA）**

     高速平面关节机器人，用于快速分拣、精密组装（如电子产品）。

3. 协作机器人（Cobot）

   - **CRX系列（CRX-10iA/L）**

     负载10kg，无安全围栏设计，支持拖拽示教，适合人机协作场景（如医疗设备组装）。

4. 伺服电机与驱动系统

   - **α、β系列伺服电机**

     标准型伺服电机（α系列高性能，β系列经济型），配套驱动器支持高响应控制。

   - **Σ-7/Σ-V系列**

     新一代伺服系统，支持EtherCAT总线通信，适用于高速高精度定位（如半导体设备）。

5. 激光加工设备

   - **FANUC Laser**

     集成激光切割/焊接功能，搭配CNC系统，适用于汽车车身三维切割与新能源电池焊接。

6. ROBODRILL 加工中心

   - **小型立式加工中心**

     高速钻孔攻丝（主轴转速24,000rpm），适合3C行业金属件加工（如手机外壳）。

7. 物联网和自动化软件

   - **FIELD System**

     工业物联网平台，实时采集设备数据，支持AI优化生产（如预测性维护）。

   - **iHMI & CNC Guide**

     人机界面软件，支持虚拟调试和离线编程，缩短设备停机时间。

8. 周边设备与解决方案

   - **视觉系统（iRVision）**

     集成2D/3D视觉引导，用于机器人定位、缺陷检测。

   - **力传感器（Force Sensor）**

     实现精密力控操作（如抛光、去毛刺）。

# 3. 应用场景

- **汽车制造**：焊接、喷涂、冲压线搬运。
- **3C电子**：精密组装、检测、高速分拣。
- **航空航天**：复杂曲面加工、复合材料处理。
- **新能源**：电池模组组装、激光焊接。



常用协议

- FOCAS(Fanuc Open CNC API Specifications)

  Fanuc FOCAS 是从 Fanuc CNC 机器收集数据的标准协议。它是一种广泛采用的工业通信协议，因为许多机床制造商使用发那科 CNC 控制器来控制他们的设备。

  FOCAS 库由 Fanuc CNC 提供，用于检索 CNC 内部的大部分信息。 Neuron 使用这些库通过以太网直接从控制器访问信息。通过 FOCAS 可获得的常见数据包括：CNC 状态（运行、空闲、警报）、零件计数信息、程序名称、编号、尺寸和修改日期、刀具和工件偏移、警报编号和文本、进给倍率、参数、位置数据 、主轴转速和模态数据等。

  - **通信方式**：以太网（TCP/IP）或光纤（HSSB）。
  - **数据类型**：轴坐标、程序号、报警代码、主轴负载等。
  - **开发要求**：需申请FANUC开发包（含DLL/头文件）。

- Ethernet/IP

  - **适用场景**：与PLC（如罗克韦尔）协同控制，需配置FANUC Ethernet/IP选项。

- MTConnect

  - **原理**：基于HTTP/XML的开源协议，标准化机床数据模型（如设备状态、切削参数）。
  - **适配方式**：通过FANUC MT-LINKi网关或第三方适配器（如Datanomix）。

- OPC UA

  - **优势**：跨平台、安全性高，支持复杂数据结构（如西门子、马扎克CNC常用）。
  - **配置**：需CNC支持OPC UA Server功能（部分机床需额外授权）。

- Modbus TCP/RTU

  - **适用场景**：低成本采集基础数据（如启停状态、报警信号），需通过PMC（可编程机床控制器）映射数据到寄存器。



# 4. References

- fanuc official: https://www.fanuc.com
- Product Brochures: https://www.fanuc.co.jp/en/product/catalog/
- Fanuc Focas Library: https://www.inventcom.net/fanuc-focas-library/general/fwlib32
- Github FANUC_Focas_API : https://github.com/wheeliar/FANUC_Focas_API
- Github fwlib: https://github.com/strangesast/fwlib

---
- Fanuc Driver: https://docs.ladder99.com/en/v0.9.2/page/drivers/fanuc/index.html
-  cnblogs 发那科 数据采集系统:  https://www.cnblogs.com/fzxxkj/p/17541124.html

