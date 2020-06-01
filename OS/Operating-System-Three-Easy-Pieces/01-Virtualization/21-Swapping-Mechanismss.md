<!--
 * @Author: JohnJeep
 * @Date: 2020-05-12 21:31:05
 * @LastEditTime: 2020-06-01 13:49:53
 * @LastEditors: Please set LastEditors
 * @Description: 物理内存之外: 机制部分(Mechanisms)
-->

### 问题(crux 0f the problem)
- HOW TO GO BEYOND PHYSICAL MEMORY?

操作系统如何利用较大，较慢的设备来透明地提供较大的虚拟地址空间的错觉(illusion)。


### 交换空间(swap space)
- 什么叫交换空间：在硬盘上开辟一片空间用于物理页(physical pages)的移入和移出。
- 交换空间的大小是非常重要的，决定了系统在某一时时刻使用的最大内存页数(the maximum number of memory pages)

### 存在位(The Present Bit)
希望将页(page)交换到(hard disk drive)中，硬件在PTE(page table entry)中查找时，如何判断该页是在内存中还是在硬盘中，则需要使用The Present Bit。若The Present Bit设置为 1， 则page存在于内存中，若The Present Bit设置为 0， 则page存在于硬盘中。

### 页错误(The Page Fault)
- 什么是页错误？
  - PTE访问不在内存中的page时，就会产生page fault。 
- OS怎么知道所需的页(page)在哪儿？
  - OS使用PTE中的某些 bit 来存储hard disk drive的地址。当OS接受到page fault时，会在PTE中查找地址，并将request发送到hard disk drive，将page读取到内存中。

### 交换什么时候发生(When Replacements Really Occur)
- 当操作系统注意到可用的页少于LW(low watermark: 低水位线)页时，将运行一个负责释放内存的后台线程。后台线程踢出页面，直到有可用的HW页面为止。
- background thread(后台线程)也称为交换守护进程(swap daemon)或页守护进程(page daemon)
- 交换算法的流程
  - 先检查是否有空闲页，而不是直接执行。若没有空闲页，通知后台分页线程按需要释放空闲页。当线程释放一些页时，会重新唤醒原来的线程，然后把需要的页面交换进内存，后台进程会继续工作。

