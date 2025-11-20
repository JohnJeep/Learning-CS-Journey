<!--
 * @Author: JohnJeep
 * @Date: 2021-02-08 21:59:39
 * @LastEditTime: 2025-11-20 11:44:29
 * @LastEditors: JohnJeep
 * @Description: 探究内存管理
-->

- [1. History(历史)](#1-history历史)
- [2. 内存管理层级](#2-内存管理层级)
- [3. 内存库作品](#3-内存库作品)
- [4. 优化内存布局](#4-优化内存布局)
  - [4.1. 分析工具](#41-分析工具)
    - [4.1.1. pahole](#411-pahole)
    - [4.1.2. structpack](#412-structpack)
    - [4.1.3. XLA](#413-xla)
    - [4.1.4. MNN推理引擎](#414-mnn推理引擎)
  - [4.2. 🛠️ 使用技巧](#42-️-使用技巧)
- [5. Bibliography(书目)](#5-bibliography书目)
- [6. Reference(参考)](#6-reference参考)


# 1. History(历史)

Doug Lea自1986年开始研究malloc算法，他的作品被称为 DL Malloc，目前linux中的glibc的malloc算法就是直接来自Doug Lea，其它平台的malloc的实现或多或少受到DL的影响。



# 2. 内存管理层级

1. OS（操作系统），Windows 系统的的 `HeapAlloc` 和 `VirtualAlloc`；
2. GUNC++/CRT 编译器的 `malloc()`
3. C++ 标准库的 `Allocator`
4. C++ 应用程序层面的内存 API 函数。





# 3. 内存库作品

- tcmalloc：稳定，占用内存更低。
- jemalloc性能更高，占用内存更高



# 4. 优化内存布局

## 4.1. 分析工具

### 4.1.1. pahole

Linux下最专业的内存布局分析工具。

`pahole` 是一个用于分析 Linux 内核中结构体大小和布局的工具，可以帮助开发者更好地理解内核数据结构占用的内存。它通过分析内核的调试信息，展示结构体成员在内存中的排列和占用空间。 

- 专门用来分析结构体和类的内存布局
- 会显示每个成员的偏移量、大小和填充空洞
- 用法：`pahole your_binary`
- 优势：能直接看到优化前后的内存布局对比

主要用法

-  **查看结构体大小**：`pahole` 可以显示特定结构体的大小（以字节为单位），这对于优化内存使用非常重要。

- **分析结构体布局**：它可以显示结构体内部成员的偏移量、大小和类型，帮助开发者理解结构体在内存中的具体排列方式。

- **识别内存浪费**：通过分析结构体的布局，开发者可以发现由于内存对齐等因素造成的“空洞”，从而进行结构体优化。 

应用场景

- **内核开发**：Linux 内核开发者使用 `pahole` 来分析和优化内核数据结构，确保它们在内存中尽可能紧凑，提高性能和效率。
- **系统分析**：系统管理员和性能分析师可以使用 `pahole` 来深入了解系统内存的分配情况，找出潜在的性能瓶颈。
- **调试**：当遇到与内存相关的 bug 时，`pahole` 可以帮助开发者快速定位问题，理解数据结构在内存中的实际状态

LWN.net: [Poke-a-hole and friends](https://lwn.net/Articles/335942/)

### 4.1.2. structpack

（开源内存布局优化神器）

- 通过解析ELF文件中的DWARF数据，分析结构体内存布局
- 会给出调整成员顺序后能节省多少内存的建议
- 适合：嵌入式系统、高性能计算、游戏开发
- GitHub地址：https://github.com/philippk/structpack

### 4.1.3. XLA

机器学习编译器

- 通过Layout类控制内存布局
- 优化维度排序、分块策略和层级平铺
- 优势：能将硬件缓存利用率提升30%以上
- 适合：深度学习模型训练优化

### 4.1.4. MNN推理引擎

推理引擎中的内存优化实践

- 采用NC4HW4数据格式优化内存布局
- 通过通道分组提高内存访问效率
- 适合：深度学习推理场景

## 4.2. 🛠️ 使用技巧

内存布局分析的步骤：

1. 如果你用C++，先用`pahole`查看类的内存布局
2. 用`structpack`分析结构体优化可能性
3. 调整成员顺序，减少填充
4. 用`sizeof`验证优化效果



**C++中的内存布局优化技巧**

- 调整结构体成员顺序（小的成员放在前面，避免填充）

- 使用`#pragma pack`控制对齐

- 例如：

  ```cpp
  struct OptimizedStruct {
      char a;
      int b;  // 4字节
      short c; // 2字节
  };
  ```

  如果顺序调整为

  ```cpp
  char, short, int
  ```

  减少填充空间

**NCHWX内存排布格式（深度学习常用）**

- 传统NCHW格式 → NCHWX（通道分组）
- 例如NCHW4：每4个通道打包存储
- 优势：提高SIMD利用率和缓存命中率



# 5. Bibliography(书目)

- STL源码剖析(侯捷)
- Small Memory Software(James Noble & Charles Weir)
- Modern C++ Design General Programming and Design Patterns Applied(Andrei Alexandrescu)



# 6. Reference(参考)
- [Doug Lea's Home Page](http://gee.cs.oswego.edu/): Malloc算法的发明者网站。
- [Wikipedia Malloc](https://en.wikipedia.org/wiki/C_dynamic_memory_allocation): 很权威的Wikipedia解释Malloc的用法。