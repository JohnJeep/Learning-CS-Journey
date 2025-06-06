<!--
 * @Author: JohnJeep
 * @Date: 2020-04-04 21:22:08
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-04-21 17:07:48
 * @Description: 网络高并发
 * Copyright (c) 2025 by John Jeep, All Rights Reserved. 
-->

# 高并发

## 多进程并发服务器

实现的思想：用父子进程(fork)和信号捕捉(signal()或sigaction)去实现。父进程关闭客户端的文件描述符，并进行信号的捕捉，回收子进程，来处理僵尸进程；子进程关闭服务端的文件描述符，进行信号的注册。



## 多线程并发服务器

- 实现的思想：设置线程的属性和线程的分离 `pthread_detach`
- 无路是采用多进程并发服务器还是多线程并发服务器，对服务器的产生的开销都比较大，一般只用一些比较小的场合，像大型的网络一般不适用，需要用到多路IO转接模型来实现。


## 多路 IO 复用

UDP 和 TCP 最基本的责任是: 将两个端系统间 IP 的传递服务扩展为运行在端系统上的两个进程之间的传递服务。将主机间传递扩展到进程间传递被称为传输层的多路复用 (transport-layer multiplexing) 与多路分解 (demultiplexing) 

- 多路IO转接模型原理：不再由应用程序(服务器)直接监听客户端，而是通过**内核**替代**应用程序**进行监听文件。
- 查看一个进程可以打开sock文件描述符的上限值 `cat /proc/sys/fs/file-max`
- 端口复用函数
  - `setsockopt()` :一般插入在 `socket()` 函数与 `bind()` 函数之间。

### select

select 和 poll 都是 UNIX/Linux 系统中的 I/O 多路复用机制，用于同时监控多个文件描述符的状态变化。

**基本工作机制**

1. 文件描述符集合：

   - 使用三个位掩码(`fd_set)`分别表示`read`、`write` 和异常事件(`expect`)
   - 每个 `bit` 代表一个文件描述符的状态

2. 工作流程：

   - 应用程序初始化 `fd_set` 集合，设置要监控的文件描述符
   - 调用`select()`系统调用
   - 内核线性扫描所有被监控的文件描述符
   - 当有事件发生或超时后，select返回
   - 应用程序需要遍历所有文件描述符找出就绪的

3. 函数原型

   ```c
   int select(int nfds, fd_set *readfds, fd_set *writefds,
              fd_set *exceptfds, struct timeval *timeout);
   ```


**特点与限制**

1. 文件描述符数量限制：
   - 通常由`FD_SETSIZE` 定义(通常是1024)
   - 无法高效处理大量连接
2. 性能问题：
   - 每次调用都需要将整个`fd_set`从用户空间拷贝到内核空间
   - 返回时需要再次拷贝修改后的`fd_set`回用户空间
   - 每次调用都需要线性扫描所有文件描述符(O(n)复杂度)
3. 重复初始化：
   - select会修改传入的 `fd_set`，因此每次调用前都需要重新初始化
4. 缺点
    - Linux中 select 监听文件描述符的的最大值为 1024 
    - 需要自定一个数据结构(数组)去遍历哪些文件描述符满足条件。
    - 每次进行操作的时候，需要将监听的集合和满足条件监听的集合进行保存，因为每次的操作会修改原有集合的值。
5. 四个辅助函数
  - `void FD_ZERO(fd_set *set)` 将set清0
  - `void FD_CLR(int fd, fd_set *set)` 将fd从set中清除出去
  - `void FD_SET(int fd, fd_set *set)` 将fd设置到set集合中去
  - `int FD_ISSET(int fd, fd_set *set)` 判断fd是否在set集合中 

### poll

**基本工作机制**

1. 文件描述符表示：

   使用 `pollfd` 结构体数组，每个元素包含：

   ```c
   struct pollfd {
       int fd;         // 文件描述符
       short events;   // 请求监控的事件
       short revents;  // 返回的事件
   };
   ```

2. 工作流程：

   - 应用程序初始化 `pollfd` 数组
   - 调用 `poll()` 系统调用
   - 内核线性扫描所有被监控的文件描述符
   - 当有事件发生或超时后，`poll` 返回
   - 应用程序需要遍历 `pollfd` 数组检查 `revents` 字段

3. 函数原型：

   ```c
   int poll(struct pollfd *fds, nfds_t nfds, int timeout);
   ```

   - `fds` 数组的首地址  
   - `nfds` 数组中元素的个数
   - `timeout` 超时时间(单位为ms级别)
     - `-1` 阻塞等待
     - `0`  立即返回，不阻塞等待
     - `>0` 等待指定的时间长度

**特点与改进**

1. **无文件描述符数量限制**：
   - 理论上只受系统资源限制
   - 可以处理比select更多的连接
2. **更灵活的事件定义**：
   - 可以监控更多类型的事件
   - 事件和返回结果分离(events和revents)
3. **性能问题**：
   - 仍然需要每次调用时传递整个结构体数组
   - 仍然需要线性扫描所有文件描述符(O(n)复杂度)
   - 但避免了select的 `fd_set` 重复初始化问题

优点
- 监听文件描述符的返回值可以超过 1024
- 实现了监听集合与返回集合的分离
- 仅在数组中搜索，范围变小了，但是效率还是比较低

### epoll

`epoll` 是Linux系统中一种高效的I/O事件通知机制，它是 select/poll 的改进版本，能够处理大量文件描述符的I/O事件，常用于构建高性能的网络服务器。

**核心原理**

1. 事件驱动模型：

   - `epoll` 采用事件驱动的方式，只有当文件描述符状态发生变化时才会通知应用程序
   - 避免了`select/poll` 需要轮询所有文件描述符的开销

2. 三个关键系统调用：

   - `epoll_create()`：创建一个epoll实例，返回文件描述符

     ```c
     // 创建一个epoll，返回值指向Linux内核中的平衡二叉树(红黑树)的树根。
     epoll_create(int size);
     ```

   - `epoll_ctl()`：向epoll实例注册、修改或删除感兴趣的文件描述符和事件

     ```c
     // 控制某个epoll监听的文件描符上的事件：注册、删除、修改。
     // op: EPOLL_CTL_ADD/ EPOLL_CTL_MOD/ EPOLL_CTL_DEL
     // event: EPOLL_IN/ EPOLL_OUT / EPOLL_ERR
     epoll_ctl(int epfd, in op, in fd, struct epoll_event *event);
     ```

   - `epoll_wait()`：等待I/O事件的发生，返回就绪的文件描述符

     ```c
     // 等待所有监控文件描述符上事件的产生，即监听epoll红黑树上事件的发生。
     // struct epoll_event *event: event为传出参数，是一个数组。
     // int maxevents: 数组的最大值  
     epoll_wait(int epfd, struct epoll_event *event, int maxevents, int timeout); 
     ```

**工作流程**

1. 初始化阶段：
   - 调用`epoll_create()`创建epoll实例
   - 通过`epoll_ctl()`注册需要监控的文件描述符和事件类型(读/写/错误等)
2. 事件等待阶段：
   - 调用`epoll_wait()`阻塞等待事件发生
   - 内核通过回调机制监控注册的文件描述符
3. 事件通知阶段：
   - 当某个文件描述符就绪时，内核将其加入就绪列表
   - `epoll_wait()`返回就绪的文件描述符和对应的事件
4. 事件处理阶段：
   - 应用程序处理就绪的文件描述符
   - 处理完毕后继续调用`epoll_wait()`等待新事件

**性能优势**

1. 高效的数据结构：
   - 使用红黑树存储所有待监控的文件描述符，查找效率高(O(log n))
   - 就绪列表使用双向链表实现，事件发生时只需将就绪项加入链表
2. 边缘触发(ET)和水平触发(LT)模式：
   - 边缘触发(ET)：只在状态变化时通知一次
   - 水平触发(LT)：只要满足条件就持续通知
   - ET模式能减少重复通知，提高效率
3. 避免不必要的拷贝：
   - 每次调用`epoll_wait()`时不需要像select/poll那样向内核传递所有文件描述符
   - 内核维护着持久的事件注册信息

**适用场景**

epoll 特别适合处理大量并发连接但活动连接比例不高的场景，如：

- Web服务器
- 即时通讯服务器
- 游戏服务器
- 其他高并发网络应用

epoll 的高效性使其成为Linux下开发高性能网络服务器的首选I/O多路复用机制。



epoll反应堆模型

- 核心思想：调用了 `libevent` 库
- 使用 `libevent` 库优点：库的底层大量采用了回调的思想，即函数指针的使用，同时也是跨平台的。

### 三种触发模式

边缘触发(Edge-Triggered, ET)和水平触发(Level-Triggered, LT)是I/O多路复用机制中两种不同的事件通知方式，它们决定了操作系统如何通知应用程序文件描述符的状态变化。

#### epoll ET

**工作原理**：

- 仅在文件描述符状态发生变化时通知一次(如从不可读变为可读，或从不可写变为可写)
- 即使缓冲区中仍有数据，如果没有新的数据到达，不会再次通知

**特点**：

1. 类似于"状态变化时单次通知"的模式
2. 应用程序必须一次性处理完所有数据(循环读取直到 EAGAIN/EWOULDBLOCK)
3. 能减少重复通知，提高效率
4. 编程复杂度较高，容易遗漏事件

**示例场景**：

- 当socket接收缓冲区从空变为非空时，只通知一次可读事件，即使之后缓冲区仍有未读数据也不会再通知

总结

- 只有client发送数据时，才会触发。
- 调用：`event = EPOLLIN | EPOLLET`
- client将大量的数据存到epoll的缓冲区中时，server只从epoll中读取一部分的数据，这部分的数据概括了缓冲区中所有数据的信息，缓冲区中剩余的数据根据需求来进行取舍。

#### epoll LT

**工作原理**：

- 只要文件描述符处于就绪状态(读缓冲区有数据可读/写缓冲区有空闲空间)，就会持续通知应用程序
- 应用程序可以不立即处理完所有数据，下次调用 `epoll_wait()` 时会再次通知

**特点**：

1. 类似于"条件持续满足就持续通知"的模式
2. 如果事件未被完全处理，下次检查时会再次报告
3. 编程模型相对简单，不容易遗漏事件
4. 可能造成不必要的重复通知
5. 系统不做任何的声明，一般默认是水平触发。
6. 一次性对 `read()` 函数进行操作，只有 `read()` 函数执行完后，才会进行水平触发。

**示例场景**：

- 当socket接收缓冲区有数据时，每次epoll_wait()都会报告可读事件，直到缓冲区被读空

#### 非阻塞IO方式

- 优点：减少 `epoll_wait` 函数调用的次数，提高效率。
- `open()` 函数，在socket套接字中不适用。
- 结合 `fcntl()` 函数和 `readn()` 函数一起使用。 `readn()` 一次性读取 `n` 个字节后才返回。
- 如何使用？
  - 使用边沿触发方式 
  - 执行过程中使用 `while(read())`
  - 调用 `fcntl(0_NOBLOCK)`

### select poll epoll对比

1. **select**:
   - 最古老的I/O多路复用机制
   - 有文件描述符数量限制(FD_SETSIZE)
   - 每次调用需要重新设置文件描述符集合
   - 线性扫描所有文件描述符，随着监控数量增加，性能明显下降
   - 每次调用都需要在用户空间和内核空间之间拷贝监控列表
   - 只支持水平触发(LT)模式
2. **poll**:
   - 解决了select的文件描述符数量限制
   - 使用pollfd结构体数组，更灵活
   - 仍然需要线性扫描所有文件描述符，随着监控数量增加，性能明显下降
   - 每次调用都需要在用户空间和内核空间之间拷贝监控列表
   - 不需要每次重新初始化数据结构
   - 只支持水平触发(LT)模式
3. **epoll**:
   - Linux特有，性能最好
   - 使用红黑树存储监控的文件描述符，效率高
   - 支持边缘触发(ET)和水平触发(LT)模式
   - 只返回就绪的文件描述符，不需要线性扫描
   - 适合高并发场景

其它方面对比

1. **数据结构**：

   - select/poll：每次调用传递完整列表

   - epoll：内核维护持久的数据结构

2. **时间复杂度**：

   - select/poll：O(n)
   - epoll：O(1)对于就绪文件描述符

3. **适用场景**：

   - select/poll：适合少量连接或跨平台应用
   - epoll：适合Linux平台上的高并发应用

