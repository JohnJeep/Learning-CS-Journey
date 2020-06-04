<!--
 * @Author: JohnJeep
 * @Date: 2020-05-13 10:25:24
 * @LastEditTime: 2020-06-04 20:33:27
 * @LastEditors: Please set LastEditors
 * @Description: CPU虚拟化
--> 

<!-- TOC -->

- [1. CPU Virtualization](#1-cpu-virtualization)
  - [1.1. 进程API调用](#11-进程api调用)
  - [进程调度（process schedule）](#进程调度process-schedule)
  - [1.2. 多级反馈队列（Multi-Level Feedback Queue）](#12-多级反馈队列multi-level-feedback-queue)

<!-- /TOC -->

# 1. CPU Virtualization
两种CPU模式
- 特权模式(privileged mode)，也叫内核模式(kernel mode)
- 用户模式(user mode)


## 1.1. 进程API调用
- 上下文切换（context switch）：当进程停止时，它的寄存器的值被保存到内存中，通过恢复这些寄存器，操作系统（OS）可以恢复运行该进程。
- 僵尸态（zombie state）：一个进程处于已退出但未清理的状态。
  > 允许其他的进程（通常是创建的父进程）检查进程的返回代码，并查看刚刚完成的进程是否成功执行。  


- `fork()` 创建一个进程
  - 创建的进程称为子进程（child）
  - 原来的进程称为父进程（parent）
  - 子进程不会从 `main()` 函数开始执行，而是直接从 `fork()` 系统调用返回，就好像是子进程调用了 `fork()`。
- `wait()` 父进程等待子进程执行完毕。
- `exec()` 进程的执行

- 重定向（redirect）原理
  - 当子进程完成创建后，shell 在调用 `exec()` 之前关闭了标准输出（standard output），打开了一个新的文件。即将要运行程序的输出结果发送到新打开的文件中，而不是直接打印在屏幕上。

- 管道原理
  - 调用系统的 `pipe()` 函数，将前一个进程的输出作为后一个进程的输入。 

- 利用时钟中断获得CPU的控制权。
 

 ## 进程调度（process schedule）
- 进程调度衡量标准（Scheduling Metrics）
  - performance（性能）
  - fairness（公平性）  
  - response time（响应时间）

- 调度算法
  - FIFO(First In, First Out)
  - 最短任务优先（SJF: Shortest Job First）：先运行最短的任务，再运行次短的任务，依次类推。是一种非抢占式调度。
  - 最短完成时间优先（STCF: Shortest Time-to-Completion First）。每当新的工作进入系统时，它就会决定剩余工作和新工作中，谁的剩余时间最少，然后调度该工作。是一种抢占式调度。
  - 轮转（Round Robin）也称为 `时间切片（time-slicing）`。
    - 原理：在一个时间片内运行一个工作，然后切换到运行队列中的下一个任务，而不是运行一个任务就结束了，它反复执行，直到所有的任务完成。
    - 注意：时间片（ time slice）的长度必须是时钟中断周期的整数倍。

- SJF, STCF调度算法优化了周转时间（turnaround time），响应时间不好。RR(Round Robin)优化了响应时间，但周转时间不好。


## 1.2. 多级反馈队列（Multi-Level Feedback Queue）
- MLFO: (Multi-Level Feedback Queue)














