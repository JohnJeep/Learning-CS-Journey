<!--
 * @Author: JohnJeep
 * @Date: 2020-08-19 19:37:29
 * @LastEditTime: 2021-08-07 23:49:01
 * @LastEditors: Windows10
 * @Description: Linux kernel学习
 * @FilePath: /linux_kernel.md
-->
<!-- TOC -->

- [1. 理解内核的秘籍](#1-%E7%90%86%E8%A7%A3%E5%86%85%E6%A0%B8%E7%9A%84%E7%A7%98%E7%B1%8D)
- [2. 基础概念](#2-%E5%9F%BA%E7%A1%80%E6%A6%82%E5%BF%B5)
- [3. 进程](#3-%E8%BF%9B%E7%A8%8B)
- [4. 参考](#4-%E5%8F%82%E8%80%83)

<!-- /TOC -->

# 1. 理解内核的秘籍
- 以一个设计者的角度来阅读内核
- 先框架再细节
- 低版本理解原理，高版本理解实现
- 动手总结，形成自己的智慧




# 2. 基础概念
- Linux是一个单内核，运行在单独的内核空间上。具有模块化设计、抢占式内核、支持内核线程、动态装载内核模块的能力，让所有事情都运行在内核态，直接调用函数，无须消息传递。

- 内核开发者通常把那些对时间要求比较高，而本身长度又比较短的函数定义成内联函数。若果一个函数较大，会被反复调用，且没有特别的时间上的限制，并赞成把它做成内联函数。

- 在内核中，为了类型的的安全和易读性，优先使用内联函数而不是复杂的宏。

- gcc内建了一条用于优化的的指令：`likely()和unlikely()`。编译器会根据这条指令对条件分支进行优化：判断该条件是经常出现还是很少出现。

- 内核中的内存都不分页，若你每用掉一个byte时，物理内存就会减少一个byte。
- 若果一个用户程序试图进行一次非法的内存访问，内核就会出现 `SIGSEGV` 信号，并结束整个进程。若内核自己非法访问了内存，则内核中会发生内存错误，导致oops。因此，在内核中，不应该去做访问非法的内存地址，引用空指针等，否则可能会死掉。


<img src="./pictures/内核源码结构.png">


# 3. 进程
- [task_struct(进程描述符)](https://blog.csdn.net/lf_2016/article/details/54347820)




# 4. 参考
- [Linux内核中双向链表的经典实现](https://www.cnblogs.com/skywang12345/p/3562146.html#a1)
- [The Linux Kernel documentation](https://www.kernel.org/doc/html/latest/): 官方Linux kernel英文API手册。
- [Linux source code Bootlin](https://elixir.bootlin.com/linux/latest/source): 在线查看Linux kernel源码。
- [What every programmer should know about memory](https://lwn.net/Articles/250967/)：博客写了关于Memory、CPU caches、Virtual memory、NUMA systems、cache optimization、multi-threaded optimizations、Memory performance tools等各个方面的知识，质量很高，需要细细的琢磨。
- [linux_kernel_wiki](https://github.com/0voice/linux_kernel_wiki): 非常丰富的 Linux 内核学习的方法，值得重点看。













