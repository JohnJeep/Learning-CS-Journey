<!--
 * @Author: JohnJeep
 * @Date: 2021-08-22 00:01:17
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-04-08 01:09:50
 * @Description: cpp optimize
 * Copyright (c) 2025 by John Jeep, All Rights Reserved. 
-->

# 1. C++ Optimize

C++ 优化的核心在于编写高效的算法、避免不必要的计算和步骤，以及选择恰当的优化设计策略。


# 2. 开销来自哪里

- 函数之间的调用。用模板和内联函数去解决。
- 虚函数调用。用 CRTP 和 variant 去解决。
- 内存分配和访问。用对象池和内存对齐去解决。
- 不必要的计算。用算法优化和编译器优化去解决。



# 3. 编写高效的 C++ 程序

让编写的程序提高运行效率的方式：
- 高效的算法。
- 避免不必要计算和步骤。
- 选择恰当的优化设计策略。


# 4. 虚函数性能开销 

虚函数会带来一定的效率损失，但在绝大多数情况下可以忽略不计，只有在极其苛刻的性能场景（如游戏引擎核心循环、高频调用的底层库）中才需要重点关注。

内存开销：
- 每个对象增加一个虚函数表指针（vptr），64 位系统下占 8 字节。
- 每个类（有虚函数的类）产生一个虚函数表（vtable），大小取决于虚函数数量（每个虚函数占一个函数指针，8 字节）。
- 对于包含大量小对象的数组（例如上百万个int+虚函数），这 8 字节可能很可观（内存占用增加 25% 以上）。


调用开销：
- 虚函数调用需要通过虚函数表进行间接调用，增加一次内存访问和一次函数调用的开销，通常比非虚函数调用慢10-20倍。相比普通成员函数（直接跳转），虚函数调用多了间接寻址：
```cpp
// 普通成员函数调用, 编译期间确定
obj.normalFunc(); // 直接调用

// 虚函数调用, 运行时确定
obj.virtualFunc(); // mov rdi, obj
                   // mov rax, [rdi]  ; 访问vptr
                   // mov rax, [rax + offset] ; 访问vtable中的函数指针
                   // call rax          ; 调用函数
```
- 在性能敏感的代码路径中（如游戏引擎核心循环、高频调用的底层库），虚函数调用的开销可能会成为性能瓶颈。尤其是在频繁调用的情况下（如每帧调用数百万次）。在这种情况下，可能需要考虑使用其他设计模式（如CRTP）来避免虚函数调用的开销。


内联优化失效：
- 虚函数调用无法被内联优化器内联，因为编译器在编译时无法确定调用哪个函数实现。
- 这意味着虚函数调用无法享受内联函数带来的性能提升，尤其是在频繁调用的情况下，可能会导致性能下降。对于非常小的函数（如`int get(){return x;}`），内联能提升几十倍性能，而虚函数会完全失去这个优势。


优化建议：
1. 先测量：用 profiler(性能分析器) 确认虚函数确实是瓶颈；Linux 下可以用 `perf`，Windows 下可以用 Visual Studio 的性能分析工具。
```bash
# 例如，使用 Linux perf 工具分析虚函数调用的性能
# 编译时加上 -g 以生成调试信息
g++ -O2 -g -o my_program my_program.cpp

# 运行程序并收集性能数据
perf record -g ./my_program

# 分析性能数据，查看虚函数调用的开销
perf report --stdio | grep virtualFunc
```
2. 优先用 variant: C++17 后，variant 是替代虚函数的最佳平衡点
3. 考虑 CRTP：当你需要给用户提供扩展点，但所有类型编译期可知
4. 保留虚函数：在模块边界、框架设计中，可读性和扩展性比1-2ns更重要

> 记住：过度消除虚函数会让代码变成模板地狱。90% 的情况下，虚函数够好；9% 用 `variant/CRTP`；1% 才需要手工优化。


# 5. References

- [Software optimization resources](https://www.agner.org/optimize/)
- 《Optimized C++》Kurt Guntheroth 大师编写的 C++ 优化书籍。
- 《Effective STL》
- 《Effective C++》
- 《Effective Modern C++》

