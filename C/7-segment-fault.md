---
title: 7-segment-fault
data: 2025-03-30 00:04:11
tags: ['C']
category: C
---

<!--
 * @Author: JohnJeep
 * @Date: 2019-07-28 21:54:53
 * @LastEditTime: 2021-05-20 23:06:25
 * @LastEditors: Please set LastEditors
 -->
# Segmentation Fault

# 1. C语言中常见的问题
- 内存重叠的处理
- 临时变量太多或 `new` 的内存没有安全释放
- 没有测试内存越界
- 指针操作不熟悉 


# 2. 概念
- 什么是段错误(segmentation fault)？
  > 程序发生了越界访问，cpu就会产生相应的异常保护，于是segmentation fault就出现了。


# 3. 产生的原因
- 什么时候会发生 `segmentation fault`?
  - 访问了不可访问的内存，这个内存区要么是不存在的，要么是受到系统保护的。
  - 错误的访问类型引起
  - 访问了不属于进程地址空间的内存
  - 内存越界，数组越界，变量类型不一致等,访问到不属于你的内存区域
  - 试图把一个整数按照字符串的方式输出
  - 栈溢出了，有 `SIGSEGV` 有时却啥都没发生

- 为什么访问 `null pointer`会发生 `segmentation fault`？
  > 进程(process) 运行时生成了一个虚拟地址(virtual address)0，硬件尝试在TLB(Translation Lookaside Buffer)中查找VPN(virtual page number)0，在TLB中没有找到，则查询页表(page table)，发现 `VPN为0` 的 `entry` 被标记为无效。遇到无效的访问，将控制权交给OS(operation system)，OS可能会使终止进程。


# 4. return返回
- `return` 语句返回时，不可指向 `栈内存` 的 `指针` 或者 `引用`，因为该内存在函数体结束时被自动销毁。


# 5. 参考
- [C语言再学习 段错误（核心已转储）](https://blog.csdn.net/qq_29350001/article/details/53780697)
- [C语言再学习](https://blog.csdn.net/qq_29350001/article/category/9267707/3) 