<!--
 * @Author: JohnJeep
 * @Date: 2020-05-12 21:31:05
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-05-31 19:59:13
 * @Description: Swapping-Mechanisms
 * Copyright (c) 2026 by John Jeep, All Rights Reserved. 
-->

## 问题(crux 0f the problem)

- HOW TO GO BEYOND PHYSICAL MEMORY?
  > 操作系统如何利用较大，较慢的设备来透明地提供较大的虚拟地址空间的错觉(illusion)。


## 交换空间(swap space)

- 什么叫交换空间：在硬盘上开辟一片空间用于物理页(physical pages)的移入和移出。
- 交换空间的大小是非常重要的，决定了系统在某一时时刻使用的最大内存页数(the maximum number of memory pages)


## 存在位(The Present Bit)

- 希望将页(page)交换到(hard disk drive)中，硬件在 PTE(page table
  entry)中查找时，如何判断该页是在内存中还是在硬盘中，则需要使用 The Present Bit。若 The Present
  Bit 设置为 1， 则 page 存在于内存中，若 The Present Bit 设置为 0， 则 page 存在于硬盘中。


## 页错误(The Page Fault)

- 什么是页错误？
  - PTE 访问不在内存中的 page 时，就会产生 page fault。 
- OS 怎么知道所需的页(page)在哪儿？
  - OS 使用 PTE 中的某些 bit 来存储 hard disk drive 的地址。当 OS 接受到 page fault 时，会在 PTE 中查找地址，并将
    request 发送到 hard disk drive，将
    page 读取到内存中。


## 交换什么时候发生(When Replacements Really Occur)

- 当操作系统注意到可用的页少于 LW(low watermark: 低水位线)页时，将运行一个负责释放内存的后台线程。后台线程踢出页面，直
  到有可用的 HW 页面为止。
- background thread(后台线程)也称为交换守护进程(swap daemon)或页守护进程(page daemon)
- 交换算法的流程
  - 先检查是否有空闲页，而不是直接执行。若没有空闲页，通知后台分页线程按需要释放空闲页。当线程释放一些页时，会重新唤醒
    原来的线程，然后把需要的页面交换进内存，后台进程会继续工作。
