# 高性能定时器

## 结构

定时器通常至少包含两个成员：

- 超时时间（相对时间或绝对时间）。
- 任务回调函数。

有时候还包含回调函数被执行时所需要传入的参数，以及是否重启定时器等信息。



## 案例

- 腾讯开源的 [libco](https://github.com/Tencent/libco) 库：是微信后台大规模使用的c/c++协程库: **时间轮**的方式。
- 开源 [Asio](https://think-async.com/Asio/) 库：红黑树最小堆算法。
- nginx：红黑树最小堆算法。
- linux 内核：Hierarchy 时间轮算法。

#  Reference

- 深入Linux C/C++ Timer定时器的实现核心原理: https://www.cnblogs.com/sunsky303/p/14154190.html
- Linux内核时钟系统和定时器实现: http://walkerdu.com/2016/07/25/linux-kernel-timer
- Reinventing the timer wheel: https://lwn.net/Articles/646950/
- Github Linux 内核高精度 httimer 实现: https://github.com/torvalds/linux/blob/master/kernel/time/hrtimer.c
- A new approach to kernel timers: https://lwn.net/Articles/152436/
- 《Linux-UNIX系统编程手册-上下册》
- 《the linux programming interface》
- 《Linux高性能服务器编程·游双》



