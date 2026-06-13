<!--
 * @Author: JohnJeep
 * @Date: 2020-04-04 09:46:51
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-06-13 16:39:48
 * @Description: Project description
 * Copyright (c) 2025 by John Jeep, All Rights Reserved. 
-->

<div align="center">

# Learning Computer Science Journey

🌱 **持续成长的计算机科学知识库**

*记录从硬件底层到应用层的学习与思考，内容持续更新，欢迎交流与建议！*

[快速导航](#-知识图谱)<br>
[开始阅读](#-编程语言)<br>
[参与交流](#-交流反馈)<br>
[成长足迹](STARCHARTS.md)<br>
[English README](README.en.md)

</div>

---

## 📖 项目简介

本项目是一份**系统化的计算机科学成长型知识库**，自 2020 年起持续维护，内容涵盖从底层硬件到上层应用的完整学习路径。

所有内容均为个人学习、工作与思考的积累，强调"知其然，更知其所以然"。

### 特色亮点

- 🏗️ **体系完整** — 芯片、汇编、C/C++、Go、操作系统、网络、数据库、云原生、机器人等多领域覆盖
- 📝 **持续更新** — 内容不断扩展，注重原理剖析与实践总结，非简单搬运
- 🎓 **课程融合** — 融合 MIT、Stanford、清华等名校课程精华
- 🔧 **实用导向** — 包含代码示例、配置、调试技巧，注重实战
- 📊 **图文并茂** — 丰富的架构图、思维导图辅助理解

---

## 🗺️ 知识图谱

```
                        ┌──────────────────────┐
                        │   Learning CS Journey │
                        └──────────┬───────────┘
           ┌───────────────┬───────┴───────┬───────────────┐
           ▼               ▼               ▼               ▼
    ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────────┐
    │  硬件 & 底层  │ │  编程语言     │ │  系统 & 网络  │ │  应用 & 前沿  │
    └──────┬──────┘ └──────┬──────┘ └──────┬──────┘ └──────┬──────┘
           │               │               │               │
    Architecture     C / C++          OS (MIT/THU)   Cloud Native
    Assembly         Go               Linux           Distributed
    Embedded         Python           Network(CS144)  AI / AIGC
    GNU              Shell/JS/TS      MySQL / Redis   Robotics (ROS2)
                                      Nginx           AutoDrive
```

---

## 📚 目录

- [硬件与底层](#-硬件与底层)
- [编程语言](#-编程语言)
- [操作系统](#-操作系统)
- [计算机网络](#-计算机网络)
- [数据结构与算法](#-数据结构与算法)
- [设计模式](#-设计模式)
- [数据库](#-数据库)
- [云原生与分布式](#-云原生与分布式)
- [AI 与前沿技术](#-ai-与前沿技术)
- [开发工具与效率](#-开发工具与效率)
- [推荐资源](#-推荐资源)
- [贡献指南](#-贡献指南)

---

## 🔩 硬件与底层

### 芯片与 CPU 架构

涵盖 **x86、ARM、NVIDIA** 三大主流架构的学习笔记，理解计算机最底层的运行原理。

### 汇编语言

- [基础汇编指令解释](Assembly/assembly.md) — 寄存器、寻址模式、常用指令集

### 嵌入式开发

STM32 系列 (F103 / F207 / F407) 嵌入式开发，包括驱动编写、外设配置、RTOS 等。

### GNU 工具链

GCC 编译器、glibc 标准库、ABI 规范等 GNU 生态核心工具。

---

## 💻 编程语言


### C 语言

C 语言是接触的第一门语言，这里重点记录工作中对 C 语言知识的深入补充——底层内存、指针、结构体、回调函数等核心主题。

<details>
<summary>展开查看完整目录</summary>

**关键字与修饰符**
- [volatile](C/6_volatile.md)
- [const](C/10_const.md)
- [restrict](C/11_restrict.md)
- [extern](C/12_extern.md)
- [static](C/38_static.md)
- [typedef](C/17_typedef.md)
- [register & auto](C/33_register_auto.md)

**指针与内存（精髓）**
- [指针](C/21_pointer.md)
- [指针函数与函数指针](C/9_pointer_function_function_pointer.md)
- [数组与指针](C/24_array_pointer.md)
- [回调函数](C/37_callback_function.md)
- [内存分布](C/30_memory.md)
- [memset](C/14_memset.md)
- [memcpy vs strncpy](C/15_strncpy_memcpy.md)
- [memcmp vs strcmp](C/16_memcmp_strcmp.md)
- [结构体与字节对齐](C/18_struct_bytealigned.md)
- [\_\_attribute\_\_((packed))](C/28___attribute__((packed)).md)
- [sizeof vs strlen](C/31_sizeof_strlen.md)

**基础与进阶**
- [字符编码](C/2_character_encoding.md)
- [字节序](C/3_byte_order.md)
- [段错误分析](C/7_segment_fault.md)
- [枚举类型](C/22_enumerations.md)
- [状态机](C/23_state_machine.md)
- [线程池](C/27_thread_pool.md)
- [动态库](C/40_dynamic_library.md)
- [编码规范](C/0_code_of_conduct.md)

</details>

### C++

C++ 知识体系庞大，从基础语法到现代 C++ 特性，从 STL 源码到内存管理，系统性地记录学习过程。

- **[C++ 基础](Cpp/Novice.md)** — 基础语法与常用特性
- **[C++ 高级](Cpp/Advance.md)** — 面向对象：封装、继承、多态；泛型编程、元编程
- **[C++ 新特性](Cpp/C11.md)**
    - [C++11](Cpp/C11.md)
    - [C++14](Cpp/C14.md)
    - [C++17](Cpp/C17.md)
    - [C++20](Cpp/C20.md)
- **[STL 标准库](Cpp/STL.md)** — 容器、迭代器、算法、仿函数，附 STL 源码分析
- **[内存管理](Cpp/MemoryManagement.md)** — 内存模型、智能指针、内存泄漏检测
- **[并发编程](Cpp/Concurrency.md)** — 多线程、锁、条件变量、[线程池](Cpp/ThreadPool.md)
- **[移动语义](Cpp/MoveSemantics.md)**
- **[智能指针](Cpp/SmartPointer.md)**
- **[编码规范](Cpp/StyleGuide.md)**

### Go 语言

从语言基础到运行时底层原理，全面覆盖 Go 语言知识体系。

- **基础**
  - [Go 基础](Go/Go.md)、[标准库](Go/GoStandardLibrary.md)、[编码风格](Go/GolangCodeConduct.md)、[测试](Go/GoTest.md)
- **进阶**
  - [CGO](Go/CGO.md)、[编译器](Go/Compiler.md)、[性能优化](Go/Performance.md)、[第三方库](Go/ThirdPackages.md)
- **底层原理**
  - [Go Internals](Go/GoInternals.md)、[GPM 调度模型](Go/GPM.md)、[Map](Go/Map.md)、[Slice](Go/Slice.md)

### 其他语言

| 语言 | 目录 | 说明 |
|------|------|------|
| **Python** | `Python/` | Python 基础与标准库 |
| **JavaScript** | `JavaScript/` | JavaScript 语言特性 |
| **TypeScript** | `TypeScript/` | TypeScript 配置与开发 |
| **Node.js** | `node.js/` | 运行时、npm 包管理 |
| **Shell** | `Shell/` | [Shell 脚本](Shell/shell.md)、tmux、crontab |
| **Qt** | `Qt/` | [Qt GUI 框架](Qt/Qt.md) 常见组件与用法 |

---

## 🖥️ 操作系统

操作系统是与底层硬件结合最紧密的课程，通过名校课程 + 经典书籍系统性学习。

### MIT 6.828

- [MIT 6.828 操作系统工程](OS/MIT-CS6.828/CS6.828.md) — 配套 xv6 实验

### OSTEP（操作系统导论）

[Operating Systems: Three Easy Pieces](http://pages.cs.wisc.edu/~remzi/OSTEP/) — 免费的操作系统启蒙之书。

<details>
<summary>展开查看详细笔记</summary>

**虚拟化 (Virtualization)**
- [CPU 虚拟化](OS/Operating-System-Three-Easy-Pieces/Virtualization/01-12-CPU-Virtualization.md)
- [地址空间](OS/Operating-System-Three-Easy-Pieces/Virtualization/13-Abstraction-Address-Space.md)
- [内存 API](OS/Operating-System-Three-Easy-Pieces/Virtualization/14-Interlude-Memory-API.md)
- [地址转换](OS/Operating-System-Three-Easy-Pieces/Virtualization/15-Address-Translation.md)
- [分段](OS/Operating-System-Three-Easy-Pieces/Virtualization/16-Segmentation.md)
- [空闲空间管理](OS/Operating-System-Three-Easy-Pieces/Virtualization/17-Free-Space-Management.md)
- [分页](OS/Operating-System-Three-Easy-Pieces/Virtualization/18-Introduction-to-Paging.md)
- [TLB](OS/Operating-System-Three-Easy-Pieces/Virtualization/19-Translation-Lookaside-Buffers.md)
- [页表](OS/Operating-System-Three-Easy-Pieces/Virtualization/20-Advanced-Page-Tables.md)
- [交换机制](OS/Operating-System-Three-Easy-Pieces/Virtualization/21-Swapping-Mechanisms.md)
- [交换策略](OS/Operating-System-Three-Easy-Pieces/Virtualization/22-Swapping-Policies.md)

**并发 (Concurrency)**
- [线程概念](OS/Operating-System-Three-Easy-Pieces/Concurrency/26-Concurrency-and-Threads.md)
- [线程 API](OS/Operating-System-Three-Easy-Pieces/Concurrency/27-Thread-API.md)
- [锁](OS/Operating-System-Three-Easy-Pieces/Concurrency/28-Thread-Lock.md)
- [条件变量](OS/Operating-System-Three-Easy-Pieces/Concurrency/29-30-Condition-Variables.md)
- [信号量](OS/Operating-System-Three-Easy-Pieces/Concurrency/31-Semaphore.md)
- [并发问题与死锁](OS/Operating-System-Three-Easy-Pieces/Concurrency/32-Common-Concurrency-Problems.md)
- [基于事件的并发](OS/Operating-System-Three-Easy-Pieces/Concurrency/33-Event-based-Concurrency.md)

</details>

### 清华大学 OS

- [清华大学操作系统课程 (2019)](https://chyyuu.gitbooks.io/os_course_info/content/)
- [uCore OS 实验指导书](https://learningos.github.io/ucore_os_webdocs/)

### 内功修炼

推荐书籍：《深入理解计算机系统 (CSAPP)》《程序员的自我修养》《老码识途》

视频：[CMU CSAPP 课程 (B 站)](https://www.bilibili.com/video/BV1iW411d7hd)

---

## 🌐 计算机网络

- [计算机网络基础](Network/NetworkPrimer.md)
- [常用术语缩写](Network/AbbrNetworkTerms.md)
- [HTTP 基础](Network/HTTP.md)
- [Wireshark 抓包分析](Network/Wireshark.md)

### Stanford CS144

课程：[CS 144: Introduction to Computer Networking](https://cs144.github.io/)

推荐书籍：
- [《计算机网络：自顶向下方法》第 8 版](http://gaia.cs.umass.edu/kurose_ross/)
- [配套 Wireshark Labs](http://gaia.cs.umass.edu/kurose_ross/wireshark.htm)

视频：[Stanford CS144 (B 站)](https://www.bilibili.com/video/BV137411Z7LR)

---

## 🧮 数据结构与算法

[数据结构与算法笔记](DataStructure/DataStructure.md) — 涵盖链表、树、栈/堆、排序、查找等经典数据结构与算法。

---

## 🏛️ 设计模式

[23 种设计模式 (C++ 实现)](DesignPattern/DesignPattern.md) — 创建型、结构型、行为型三大类，配合 UML 类图辅助理解。

---

## 🗄️ 数据库

| 数据库 | 内容 |
|--------|------|
| **MySQL** | [MySQL 基础](MySQL/MySQL.md) — SQL 优化、连接池、架构原理 |
| **Redis** | [Redis 基础](Redis/Redis.md) — 数据结构、集群、源码分析 |
| **Nginx** | [Nginx 基础](Nginx/Nginx.md) — 配置、反向代理、负载均衡 |

---

## ☁️ 云原生与分布式

### 云原生

| 技术栈 | 笔记链接 |
|--------|----------|
| **容器化** | Docker<br>Kubernetes |
| **服务通信** | [gRPC](CloudNative/gRPC/)<br>[Protobuf](CloudNative/Probobuf/)<br>[MQTT](CloudNative/Mqtt.md) |
| **消息队列** | [Kafka](CloudNative/Kafka/)<br>[RocketMQ](CloudNative/RocketMQ/) |
| **服务治理** | [Zookeeper](CloudNative/Zookeeper/)<br>[Actor 模型](CloudNative/Actor.md) |
| **监控** | [InfluxDB](CloudNative/Influxdb.md)<br>[Jenkins CI/CD](CloudNative/Jenkins/) |

### 分布式系统

- MIT 6.824 分布式系统课程 — [B 站视频](https://www.bilibili.com/video/BV1qk4y197bB)
- 推荐书籍：《数据密集型应用系统设计 (DDIA)》
- 微服务、API Gateway、Keepalived

---

## 🤖 AI 与前沿技术

### 人工智能

- [AIGC](AI/AIGC.md) — 生成式 AI 技术概览
- [Ollama](AI/Ollama.md) — 本地大模型部署与使用

### 自动驾驶

- [Autoware](AutoDrive/Autoware.md) — 开源自动驾驶框架


### 机器人

ROS2 完整技术栈，从基础到实战：

- **核心框架** — ROS2 基础、DDS 通信、Launch、参数管理
- **导航与感知** — SLAM、Nav2 导航、激光雷达、点云处理
- **仿真与控制** — Gazebo 仿真、行为树、ROSControl、电机驱动
- **建模** — URDF、tf 坐标变换、传感器接入

---

## 🐧 Linux 系统

- [Linux 基础](Linux/linux-primer.md) — 文件系统、用户管理、常用命令
- [编译链接原理](Linux/compile-link.md)
- [GDB 调试](Linux/gdb.md)
- [VIM](Linux/vim.md)
- [CMake](Linux/CMake/cmake-tutorial.md)
- [Makefile](Linux/CMake/Makefile.md)
- [inode 分析](Linux/inode.md)
- [Linux 系统编程](Linux/system-program.md)
- [常用工具集](Linux/linux-tools.md)

---

## 🔧 开发工具与效率

### 版本控制

- [Git 从入门到内部原理](Git-SVN/Git.md)
- [SVN 工具](Git-SVN/SVN.md)
- [Git 安装与配置](Git-SVN/Install.md)
- [开发协作指南](Git-SVN/Develop.md)
- [常见问题](Git-SVN/FAQ.md)
- [提交规范](Git-SVN/commit-convention.md)
- [CI/CD Pipeline](Git-SVN/Pipeline.md)
- [Git Internals](Git-SVN/Internals.md)

### 工业协议

- Modbus
- Omron
- Siemens
- Beckhoff
- Fanuc
- WebRTC
- SMTP
- 以及其他通信协议

### 常用工具

| 工具 | 链接 |
|------|------|
| VSCode | [快捷键与技巧](StudyTool/VSCode.md) |
| Jetbrains | [插件与使用](StudyTool/JetbrainsPlugins.md) |
| draw.io | [快捷键](StudyTool/draw.io.md) |
| Windows 效率 | [Windows 快捷键](StudyTool/Windows10.md) |
| 平板工具 | [iPad 工具清单](StudyTool/iPad-tools.md) |
| 学习资源 | [常用网站汇总](StudyTool/WebsiteReferences.md) |

### 文档与知识管理

- [Typora 使用说明](Markdown/TyporaMarkdown.md)
- [LaTeX 公式](Markdown/LaTex.md)
- [希腊字母表](Markdown/GreekAlphabet.md)
- [Jupyter 使用说明](Markdown/Jupyter.md)
- [Jupyter Notebook 示例](Markdown/Jupyter.ipynb)

### 仓库维护脚本

- [add_frontmatter.py](add_frontmatter.py): 批量补充文档头部元信息
- [clean.bat](clean.bat): Windows 环境清理脚本

---

## 📖 推荐资源

### 自学路线

- [cs_study_plan](https://github.com/spring2go/cs_study_plan) — 硬核计算机科学自学计划
- [TeachYourselfCS-CN](https://github.com/keithnull/TeachYourselfCS-CN/blob/master/TeachYourselfCS-CN.md)
  — 自学 CS 推荐课程与书籍

### 推荐书籍

| 领域 | 书籍 |
|------|------|
| 计算机系统 | 《深入理解计算机系统》《程序员的自我修养》 |
| 操作系统 | 《Operating Systems: Three Easy Pieces》 |
| 计算机网络 | 《计算机网络：自顶向下方法》 |
| 分布式 | 《数据密集型应用系统设计》 |

---

## 💬 交流反馈

欢迎通过 Issue、Discussions 或邮件交流建议与想法！提交前请阅读 [文档写作约定](convention.md)。

提问推荐阅读：
- [《提问的智慧》](https://github.com/ryanhanwu/How-To-Ask-Questions-The-Smart-Way)
- [《如何向开源社区提问题》](https://github.com/seajs/seajs/issues/545)

---

## 📜 License

Copyright (c) 2026 by JohnJeep. All Rights Reserved.

---

<div align="center">

**如果这个项目对你有帮助，请给一个 Star 支持一下！**

</div>
