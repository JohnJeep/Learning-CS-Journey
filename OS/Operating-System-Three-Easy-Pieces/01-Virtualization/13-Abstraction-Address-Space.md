---
title: 13-Abstraction-Address-Space
data: 2025-03-30 00:04:11
tags: ['01-Virtualization']
category: 01-Virtualization
---

<!--
 * @Author: JohnJeep
 * @Date: 2020-05-13 10:25:24
 * @LastEditTime: 2020-08-11 20:02:21
 * @LastEditors: Please set LastEditors
 * @Description: 抽象的地址空间笔记
--> 

# 抽象的地址空间 (Abstraction Address Space)

## 问题？
1. 如何管理可用空间？
2. 空间不足时哪些页面该释放？


## 基础
- 概念
  - 操作系统需要提供一个易于使用的物理内存抽象，叫做地址空间（正在运行的程序在系统中的内存视图）。
  - 一个进程的地址空间包含运行程序的所有状态。


- 虚拟化内存的三个作用
  - 透明(transparency): 操作系统提供的假象不应该被应用程序看破。程序不应该感知到内存被虚拟化，让它自认为拥有自己的私有物理内存。
  - 效率(efficiency): 在时间和空间上提高效率
  - 保护(protection): 确保进程受到保护，不受其它进程的影响，操作系统本身也不受进程影响。保护能够在进程之间提供隔离(isolation)，使之每个进程都能在自己独立的环境中运行，避免出错或恶意进程的影响。


- **注意** 
  - 虚拟地址只提供地址如何在内存中分布的假象，只有操作系统（和硬件）才知道物理地址。
  - 从用户的角度（程序员）看到内存中的所有地址都是**虚拟地址**


- 总结
  - 虚拟内存系统为程序提供一个巨大的、稀疏的、私有的地址空间的假象，保存了程序的所有指令和数据。
  - 操作系统在专门硬件的帮助下，通过每一个虚拟内存的索引，将其转化为物理地址，物理内存根据获得的物理地址去获取所需的信息。 
   
## 命令
- free: 显示系统中的可用和已用内存量
- pmap: 查看进程的内存映像信息
  > 参数解释
  - Address：进程所占的地址空间
  - Kbytes：该虚拟段的大小
  - RSS：设备号（主设备：次设备）
  - Anon：设备的节点号，0表示没有节点与内存相对应
  - Locked：是否允许swapped
  - Mode 权限：r=read, w=write, x=execute, s=shared, p=private(copy on write)
  - Mapping：bash 对应的映像文件名 
  - Resident ：表示在内存中驻留的段的空间   
  - shared ：表示这些北分配的内存是被系统中其他进程共享的。    
  - private ：表示只能被该进程使用的空间大小。你可以发现share的空间不具有 private的属性。
  - Prstat －LP 的输出的意义是：
  - size：就是该进程占用的地址空间。
  - RSS：实际被分配的内存的大小
- 查看进程PID
  - pgep
  - ps aux | grep ...   
   
