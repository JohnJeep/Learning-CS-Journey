<!--
 * @Author: JohnJeep
 * @Date: 2025-04-01 00:40:42
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-04-04 19:13:40
 * @Description: linux performance analysis
 * Copyright (c) 2025 by John Jeep, All Rights Reserved. 
-->
# 1. Performance optimization(性能优化)

## 1.1. 怎样做性能优化？

在做性能优化之前，需要了解性能**优化指标**。从应用负载的视角来看，有两个核心指标：**吞吐量和延时**，这两个指标直接影响了产品终端的用户体验。从系统资源的角度看，有**资源利用率、饱和度**等指标。

性能问题的本质，就是系统资源已经达到瓶颈，但请求的处理却还不够快，无法支撑更多的请求。而性能分析，就是找出应用或系统的瓶颈，并设法去避免或者缓解它们，从而更高效地利用系统资源处理更多的请求。

从运行时性能的开销和编译时性能的开销角度思考

消除运行期的开销？

## 1.2. 如何去分析

如何去思考去分析性能，该从哪些方面入手？

1. 当前应用程序或系统有哪些指标可以衡量性能？
2. 怎么样去设置应用程序或系统的性能指标？
3. 使用什么样的性能工具来观察指标？
4. 导致这些指标变化的因素是什么？
5. 怎样进行性能基准测试？
6. 怎样进行性能分析定位瓶颈？
7. 性能监控和告警是怎样的？

学习的黄金准则

1. 勤思考
2. 多反思
3. 善总结
4. 多问为什么

**1 分钟之内在命令行模式下用已有的 Linux 标准工具进行性能优化检测。**

在 1 分钟之内只需要通过运行下面的 10 个命令就可以对系统资源使用和运行进程有一个很高程度的了解，寻找错误信息和饱和度指标，显示为请求队列的长度，或者等待时长、显示资源利用率。

> 饱和度是指一个资源已经超过了它自己的负荷能力。

```
uptime
dmesg | tail
vmstat 1
mpstat -P ALL 1
pidstat 1
iostat -xz 1
free -m
sar -n DEV 1
sar -n TCP,ETCP 1
top
```

有些命令需要安装 `sysstat` 工具包。这些命令展示的指标会帮助你完成一些 USE（Utilization，Saturation，Errors） 方法：定位性能瓶颈的方法论。包括了检查使用率（Utilization），饱和度（Saturation），所有资源（比如 CPU，内存，磁盘等）的错误指标（Errors）。同样也要关注你什么时候检查和排除一个资源问题，因为通过排除可以缩小分析范围，同时也指导了任何后续的检查。

### 1.2.1. 工具

**性能工具选择的黄金准则：一个正确的选择胜过千百次的努力。** 选用合适的性能工具，可以大大简化整个性能优化的过程。

性能分析可以用哪些工具来衡量？

下图是常见的性能分析工具，来自性能领域的大师布伦丹·格雷格（Brendan Gregg）绘制，这个图是 Linux 性能分析最重要的参考资料之一，它告诉你，在 Linux 不同子系统出现性能问题后，应该用什么样的工具来观测和分析。

<img src="figures/linux_observability_tools.png" style="zoom: 33%;" >

优化工具的使用具体请看：[Linux 下常用系统性能优化工具](./performance-tools.md)

### 1.2.2. 代码编写

优化编写的代码。

## 1.3. 优化到多少？

## 1.4. 其它

```cpp
打印行和文件：stream(__FILE__, __LINE__)
```

判断是运行时变量还是编译时变量？

​    使用 static_assert()

应用：

​    替换宏（为什么要替换宏？）

​    宏运行在什么时候？（编译？运行？）

# 2. CPU

load average（平均负载）：单位时间内，系统处于可运行状态（runnable state）或不可中断状态（uninterruptable state）的平均进程数，与 CPU 使用率并没有直接关系。

可运行状态的进程：正在使用 CPU 或正在等待 CPU 的进程。也就是我们常用 ps 命令看到的，处于 R 状态（Running 或 Runnable）的进程。

不可中断状态的进程：正处于内核态关键流程中的进程，并且这些流程是不可打断的。比如最常见的是正在等待磁盘（disk），也就是我们在 ps 命令中看到的 D 状态（Uninterruptible Sleep，也称为 Disk Sleep）的进程。

比如，当一个进程向磁盘读写数据时，为了保证数据的一致性，在得到磁盘回复前，它是不能被其他进程或者中断打断的，这个时候的进程就处于不可中断状态。如果此时的进程被打断了，就容易出现磁盘数据与进程数据不一致的问题。所以，不可中断状态实际上是系统对进程和硬件设备的一种保护机制。

正确区分平均负载和 CPU 使用率

CPU 使用率指单位时间内 CPU 繁忙情况的统计，跟平均负载并不一定完全对应。比如下面的情况：

- CPU 密集型进程，使用大量 CPU 会导致平均负载升高，此时这两者是一致的；
- I/O 密集型进程，等待 I/O 也会导致平均负载升高，但 CPU 使用率不一定很高；
- 大量等待 CPU 的进程调度也会导致平均负载升高，此时的 CPU 使用率也会比较高。



# 3. Memory



# 4. I/O



# 5. Network



# 6. References

- [Linux Performance](http://www.brendangregg.com/linuxperf.html)

- [如何 60 秒内进行 Linux 性能分析](https://mp.weixin.qq.com/s?__biz=MzI0OTA3OTM4MA==&mid=2455110627&idx=1&sn=6452b61a1aa9126cd6874b6f731f03a1&chksm=fe3450d8c943d9ce389372f9da4e5a1c54d6419b98144cc346ccfafa175758901fe68cf99a73&mpshare=1&scene=24&srcid=&sharer_sharetime=1590076651796&sharer_shareid=1813da429599d3785585eac965f1aa77&key=1cb409b1e6845731cbb675abe3379f22acb3e7524678737985c9ea500376d57bbcc18d4b8e7489b2d6893af87959cedd648921fdf755fdf7e84cfa01839bda54e39dc3385f15d8a5536bd178abf3d05aa09fe562f4c63c6aa8caf0cd6d98656ffe9f1bb3464f30260a244d566cf1059e19ef198ee743d49e80051e78a222434d&ascene=14&uin=MTE2MDU5MjIzNA%3D%3D&devicetype=Windows+10+x64&version=6300002f&lang=zh_CN&exportkey=A6Io2cPWUakbyRI7AlfF3dA%3D&pass_ticket=aUFZUParu2js4nQ7AaFdbqYXSULD4Aap4Fv2P64VAtlM%2FsR52EPLWAmjVTjvWw97&wx_header=0)

- [CodeSheep 列出常用监控工具](https://mp.weixin.qq.com/s?__biz=MzU4ODI1MjA3NQ==&mid=2247495793&idx=1&sn=a46b6570280594552e5d711942f72eb0&chksm=fddd26b5caaaafa303b282a258b712a461f7a453af9d9c6737515f8edc8cf74d5c57a6a2e94d&scene=126&&sessionid=1643247338#rd)

- [如何查看 Linux 服务器性能参数指标？](https://mp.weixin.qq.com/s/8PShiKil_rOiIZbH4C95HA)


- [面试官：如何优化你的程序](https://mp.weixin.qq.com/s/S46POYxx4QQQuubpGwMaxg)
- [简书：Linux 问题故障定位的小技巧](https://www.jianshu.com/p/0bbac570fa4c)