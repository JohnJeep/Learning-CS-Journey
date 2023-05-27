<!--
 * @Author: JohnJeep
 * @Date: Wednesday, August 19th 2020, 18:58:33 PM
 * @LastEditors: JohnJeep
 * @LastEditTime: 2023-05-27 15:40:16
 * @Description: Linux kernal 学习
 * Copyright (c) 2022 by JohnJeep, All Rights Reserved. 
-->

<!-- TOC -->

- [1. 理解内核的秘籍](#1-理解内核的秘籍)
- [2. 如何学习内核](#2-如何学习内核)
  - [2.1. 核心的东西：最基础、最原始的概念。](#21-核心的东西最基础最原始的概念)
- [2. Linux 内核体系结构](#2-linux-内核体系结构)
  - [2.1. 内核模式与体系结构](#21-内核模式与体系结构)
  - [2.2. struct](#22-struct)
  - [2.3. 内核中断](#23-内核中断)
  - [2.4. 内核进程](#24-内核进程)
- [2. Linux 内核源码结构](#2-linux-内核源码结构)
  - [2.1. Linux 内核源码宏观结构](#21-linux-内核源码宏观结构)
  - [2.2. Linux 内核源码各级源码分类](#22-linux-内核源码各级源码分类)
  - [2.3. Linux 内核源码解析第一刀](#23-linux-内核源码解析第一刀)
- [2. Linux 内核引导程序](#2-linux-内核引导程序)
  - [2.1. Linux 内核启动程序分析](#21-linux-内核启动程序分析)
  - [2.2. Linux 内核初始化程序分析](#22-linux-内核初始化程序分析)
  - [2.3. Linux 第一个进程分析](#23-linux-第一个进程分析)
- [2. The Linux Storage Stack Diagram](#2-the-linux-storage-stack-diagram)
- [3. Reference](#3-reference)

<!-- /TOC -->


# 1. 理解内核的秘籍

- 以一个设计者的角度来阅读内核
- 先框架再细节
- 低版本理解原理，高版本理解实现
- 动手总结，形成自己的智慧
- 不要走弯路，直接学你要学的东西

---------------------------------------------------------------------

# 2. 如何学习内核

Linux 内核学习分为几个阶段

1. 了解操作系统基本概念。
2. 了解 Linux 内核机制，大的框架和架构。
3. 了解 Linux 内核编译环境。下载一套源码，更换源码。
4. 研读内核源码，选择自己感兴趣的方向入手。比如：调度、虚拟化、网络、内存、存储等。
5. 尝试 Linux 内核模块编写。
6. 尝试 Linux 内核模块编译调试。
7. 确定个人发展方向。
   - 内核驱动方向开发（嵌入式）
   - 内核网络方向开发（云计算）
   - 内核虚拟化（云计算）
   - Linux 应用编程

## 2.1. 核心的东西：最基础、最原始的概念。

-------------------

dmesg

menuconfig

# 2. Linux 内核体系结构

- Linux是一个单内核，运行在单独的内核空间上。具有模块化设计、抢占式内核、支持内核线程、动态装载内核模块的能力，让所有事情都运行在内核态，直接调用函数，无须消息传递。

- 内核开发者通常把那些对时间要求比较高，而本身长度又比较短的函数定义成内联函数。若果一个函数较大，会被反复调用，且没有特别的时间上的限制，并赞成把它做成内联函数。

- 在内核中，为了类型的的安全和易读性，优先使用内联函数而不是复杂的宏。

- gcc 内建了一条用于优化的的指令：`likely()和 unlikely()`。编译器会根据这条指令对条件分支进行优化：判断该条件是经常出现还是很少出现。

- 内核中的内存都不分页，若你每用掉一个byte时，物理内存就会减少一个byte。

- 若果一个用户程序试图进行一次非法的内存访问，内核就会出现 `SIGSEGV` 信号，并结束整个进程。若内核自己非法访问了内存，则内核中会发生内存错误，导致oops。因此，在内核中，不应该去做访问非法的内存地址，引用空指针等，否则可能会死掉。

<img src="./pictures/内核源码结构.png">

Linux 内核架构

<img src="./pictures/linux内核架构.png">

**Linux Architecture and features**

<img src="pictures/Linux_kernel_map.svg">



## 2.1. 内核模式与体系结构

操作系统的工作方式

1. 操作系统从用户态态切换到内核态，即用户应用程序到内核程序。
2. 实现操作系统的系统调用。
3. 应用操作系统提供的底层函数，进行功能的实现。
4. 从内核态切换到用户态。

操作系统内核中各级模块之间的关系

- Linux内核模块整体分为：进程调度模块、内存管理模块、文件系统模块、进程间通信模块、驱动管理模块。
- 每个模块之间的关系
  1. 内存管理和驱动管理模块 。
  2. 虚拟内存的缓存和回存机制。
  3. 虚拟文件系统 (VFS) 把硬件当成文件来使用。

操作系统结构的独立性

- 为什么要把 Linux 内核分成管理层和实现层：易于代码的升级和维护。
- 高版本内核与低版本内核的区别
  1. 内核驱动的种类变多了，但内核驱动的管理模式并没有发生巨大的改变，比如：一段时间的三个跳段：零散型、分层型、设备树（Android操作系统）。
  2. 进程的调度算法发生了改变，但进程的管理方式没有发生巨大的改变。

## 2.2. struct

内核中常见的 struct 结构体

- task_struct
- mm_task

## 2.3. 内核中断

## 2.4. 内核进程

task_struct(进程描述符：https://blog.csdn.net/lf_2016/article/details/54347820

每一个进程都有一个 task_struct。

# 2. Linux 内核源码结构



## 2.1. Linux 内核源码宏观结构



## 2.2. Linux 内核源码各级源码分类



## 2.3. Linux 内核源码解析第一刀



# 2. Linux 内核引导程序



## 2.1. Linux 内核启动程序分析



## 2.2. Linux 内核初始化程序分析



## 2.3. Linux 第一个进程分析





# 2. The Linux Storage Stack Diagram

<img src="pictures/The_Linux_Storage_Stack_Diagram.svg">

来源：https://en.wikipedia.org/wiki/Linux_kernel#/media/File:The_Linux_Storage_Stack_Diagram.svg





# 3. Reference

- 英文WIKI: https://en.wikipedia.org/wiki/Linux_kernel

- Wiki Linux kernel version history: https://en.wikipedia.org/wiki/Linux_kernel_version_history

- 维基百科中文解释Linux内核: https://zh.wikipedia.org/wiki/Linux%E5%86%85%E6%A0%B8

- 官方 Linux kernel 英文 API 手册: https://www.kernel.org/doc/html/latest

- 在线查看Linux kernel 源码: https://elixir.bootlin.com/linux/latest/source

- 在线man-pages手册: https://man7.org/linux/man-pages/index.html 

  由 man-pages 的维护者 Michael Kerrisk 维护的在线 man-pages 手册。其中除了 man-pages 手册外，还有许多丰富的内容，像 `The Linux Programming Interface` 等，值得日常重点查阅。

- 分享 Linux 内核开发的一些文章，很最权威，内容很优质: https://lwn.net/Kernel

- Github 上零声学院开源的 Linux 内核学习的方法，很全面: https://github.com/0voice/linux_kernel_wiki

- Linux 优化大师--布伦丹·格雷格的网站: http://www.brendangregg.com/index.html

- Linux操作系统内核学习: https://ty-chen.github.io/categories

  作者自己搭建的一个博客，里面记录了自己学习 Linux 内核方面的一些知识点。

- Linux内核中双向链表的经典实现: https://www.cnblogs.com/skywang12345/p/3562146.html

- What every programmer should know about memory: https://lwn.net/Articles/250967

  <font color=red>博客写了关于 Memory、CPU caches、Virtual memory、NUMA systems、cache optimization、multi-threaded optimizations、Memory performance tools 等各个方面的知识，质量很高，需要细细的琢磨。</font>

- Linux0号进程，1号进程，2号进程: https://cloud.tencent.com/developer/article/1603977
