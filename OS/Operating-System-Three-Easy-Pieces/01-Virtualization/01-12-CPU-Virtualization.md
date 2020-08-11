<!--
 * @Author: JohnJeep
 * @Date: 2020-05-13 10:25:24
 * @LastEditTime: 2020-08-11 20:36:26
 * @LastEditors: Please set LastEditors
 * @Description: CPU虚拟化
--> 
# 1. CPU Virtualization
- 两种CPU模式
  - 特权模式(privileged mode)，也叫内核模式(kernel mode)
  - 用户模式(user mode)


## 进程API调用
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


## 多级反馈队列（Multi-Level Feedback Queue）
- MLFO(Multi-Level Feedback Queue)：利用反馈信息来决定任务的优先级。
- MLFQ中有许多独立的队列，每个队列有不同的优先级。任何时刻，一个工作只能存在一个队列中，MLFQ总是优先执行高优先级的工作。
- MLFQ调度的规则
  - 如果 A 的优先级 > B 的优先级，则运行A，不运行B。
  - 如果 A 的优先级 = B 的优先级，则轮转运行A和B。
  - 一个任务进入系统时，被放置在最高优先级。
  - 一旦任务在给定的某一级别上用完了时间分配（无论它放弃了CPU多少次），其优先级就会降低（即，它移动到下一个队列）。
  - 每经过一个时间周期 S后，将所有的任务放入到最高优先级队列中。
 - 调度算法的缺点
   - 会产生饿死的问题，长时间的任务永远无法得到CPU。
   - 用户可能会重写程序，来愚弄调度器，让它给你远超公平的资源。

- 使用 `nice` 命令行工具，可以增加或降低工作的优先级，从而增加或降低任务在某个时刻运行的机会。


## Proportional Share（比例份额）
- proportional-share scheduler（比例份额调度器）也叫fair-share scheduler（公平份额调度器）。
- 原理：调度器确保每个任务获得一定比例的CPU时间，而不是优化周转时间或响应时间。

- 两种调度算法
  - stride scheduling（步长调度）：系统中的任务都有自己的步长，这个值与票数值（number of tickets ）成反比。
  - 彩票调度（ticket scheduling）：分配票数的方法采用随机数的方式。不需要对每个进程记录全局的状态，只需要记录总票数就可以了。


## Multiprocessor Scheduling 
- CPU从缓存中读取数据的原则
  - 程序第一次读取数据时，数据时放在内存中，花费的时间比较长。处理器期望该数据也许会再次使用，将数据放在缓存中。如果之后的时间程序再次使用同样的数据，CPU会先查找缓存。如果找到的数据就在缓存中，得到的数据块的多，程序运行的也很快。

- 多CPU中解决数据缓存一致性（cache coherence）的问题。
  - 通过监控内存访问。
  - 在总线系统中，采用总线窥探（bus snooping）。
   > 每个缓存都通过监听链接所有缓存和内存的总线，来发现内存的访问。若CPU发现缓存中的数据更新了，会将原来的数据从缓存中移除或修改位新的值。

- 多个CPU之间访问共享数据或数据结构时，需要使用 `互斥锁`，来确保数据的正确性。

- 缓存亲和度（Cache Affinity）
  > 当在特定CPU上运行时，在CPU的缓存中建立一个公平位状态（fair bit of state）。下次运行该进程在相同的CPU上时，由于缓存的数据而执行的更快。相反，在不同的CPU上执行，由于需要重新加载数据而变得较慢。因此，多处理调度器考虑到缓存的这种亲和性，并尽可能得将进程保持在同一个CPU上。


- 单队列调度（Single-Queue Scheduling）
- 多队列调度（Multi-Queue Scheduling）