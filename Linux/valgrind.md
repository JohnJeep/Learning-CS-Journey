<!--
 * @Author: JohnJeep
 * @Date: 2021-01-05 22:07:57
 * @LastEditTime: 2023-07-25 15:33:53
 * @LastEditors: JohnJeep
 * @Description: Valgrind工具使用
-->

<!-- TOC -->

- [1. 简介](#1-简介)
- [2. 用法](#2-用法)
  - [2.1. Valgrind 工具详解](#21-valgrind-工具详解)
- [3. Options](#3-options)
- [4. References](#4-references)

<!-- /TOC -->

# 1. 简介

Valgrind 是 Linux 下进行内存泄露检测和性能分析的工具。

# 2. 用法
- Valgrind包含下列工具
  1. Memcheck：是一个内存错误检测器(detector)，检查程序中的内存问题，如泄漏、越界、非法指针等，让你的程序更正确。
  2. Cachegrind：是一Cache和分支预测分析器(branch-prediction profiler)，分析CPU的cache命中率、丢失率，用于进行代码优化，让你的程序运行的更快。
  3. Callgrind：是一个图形化调用生成Cache的分析器(profiler)，检测程序代码的运行时间和调用过程，以及分析程序性能。
  4. Helgrind：是一个线程错误检测器(detector)，用于检查多线程程序的竞态条件。
  5. DRD：是一个线程错误检测器(detector)，跟 Helgrind 工具是相似的，但是它是使用不用的分析技术去找到很多不同的问题。
  6. Massif：是一个堆(heap)分析器(profiler)，分析程序中使用了多少堆内存等信息，让你的程序使用更少的内存。
  7. DHAT(dynamic heap analysis tool)：是一个动态堆分析工具，让你理解内存块的生命周期(block lifetimes)，块的利用率(block utilisation)以及内存分布的效率(layout inefficiencies)。
  8. BBV：是一个实验性的 SimPoint 基本的 vector 产生器(generator)。可以做电脑架构的研究和开发。
  9. Lackey：是一个很小的 example tool，阐述了一些基本的指令。
  10. Nulgrind：是Valgrind中最小的工具，不能分析(analysis)或 仪器测量(instrumentation)，只能用于测试。
  
  > Valgrind中使用不同的工具，则是通过命令：`valgrand --tool=name` 来指定调用， `name` 为工具的名称，当不指定tool参数时默认是 `--tool=memcheck`。
  

## 2.1. Valgrind 工具详解
* Memcheck
  
  Memcheck 是最常用的工具，用来检测程序中出现的内存问题，所有对内存的读写都会被检测到，一切对 malloc、free、new、delete 的调用都会被捕获，但是它也不能检测静态分配或 stack上 超出数组 read/write 的范围(`Memcheck cannot detect every memory error your program has. For example, it can't detect out-of-range reads or writes to arrays that are allocated statically or on the stack. But it should detect many errors that could crash your program (eg. cause a segmentation fault)`)。所以，它能检测以下问题：
  
  1. Use of uninitialised memory：使用未初始化的内存。
  2. Reading/writing memory after it has been free ：读/写释放后的内存块 。
  3. Reading/writing off the end of malloc blocks：读/写超出 malloc 分配的内存块。
  4. Reading/writing inappropriate areas on the stack：栈中读写不适当的区域。
  5. Memory leaks – where pointers to malloc blocks are lost forever：内存泄漏，指向一块内存的指针永远丢失。
  6. Mismatched use of malloc/new/new [] vs free/delete/delete [] ：不正确的malloc/free或new/delete匹配。
  7. Overlapping src and dst pointers in memcpy() and related functions)： memcpy() 相关函数中的 dst 和 src 指针重叠。
  
  > The stack trace tells you where the leaked memory was allocated. Memcheck cannot tell you why the memory leaked, unfortunately. 
  
  - 这些问题往往是 C/C++ 程序员最头疼的问题，Memcheck 能在这里帮上大忙。例如：
    
    ```c
    #include <stdlib.h>  
    #include <malloc.h>  
    #include <string.h>  
    
    void test()  
    {  
        int *ptr = malloc(sizeof(int)*10);  
        ptr[10] = 7; // 内存越界  
        memcpy(ptr +1, ptr, 5); // 踩内存  
    
        free(ptr);   
        free(ptr);// 重复释放  
    
        int *p1;  
        *p1 = 1; // 非法指针  
    }  
    
    int main(void)  
    {  
        test();  
        return 0;  
    }  
    ```

valgrind 的检测信息将内存泄漏分为如下几类：

- definitely lost：确定产生内存泄漏
- indirectly lost：间接产生内存泄漏
- possibly lost：可能存在内存泄漏
- still reachable：即使在程序结束时候，仍然有指针在指向该块内存，常见于全局变量



# 3. Options
```sh
- --track-origins=yes 生成更多的信息，找到条件跳转或move 指令 问题的原始出处
- --num-callers  让stack trace 范围更大
```

Clang-Tidy 和 CLazy 对你的代码进行静态检查


# 4. References
- [valgrind官方文档说明](https://www.valgrind.org/downloads/current.html)
- [Stack overflow解释：How to install valgrind good?](https://stackoverflow.com/questions/24935217/how-to-install-valgrind-good/51671524)
- [valgrind的介绍、安装和使用](https://blog.csdn.net/justheretobe/article/details/52986461)
- [自动生成Makefile](https://blog.csdn.net/initphp/article/details/43705765#%E5%85%B3%E4%BA%8EAutotools)
- [valgrind 内存泄漏分析 - 广漠飘羽 - 博客园](https://www.cnblogs.com/gmpy/p/14778243.html)