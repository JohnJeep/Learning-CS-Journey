## 目标

gRPC C++ 源码的质量很高，学习 Google 工程师门优秀的抽象和设计能力。



## 大量高级抽象结构

gRPC 的源码为了简化异步代码的编写，同时为了更好的代码复用。设计了许多高级的数据结构。如

- grpc_closure
- grpc_closure_scheduler
- ExecCtx
- grpc_combiner
- grpc_completion_queue

## 大量设计模式

为了提供跨平台能力，grpc核心代码采用了bridge设计模式，因此可以看到各种vtable,这也为阅读源代码增加了困难。

## 异步编程

grpc核心库采用reactor设计模式，如grpc_closure就是为了方便异步编程而设计的数据结构。



## 高性能

出于性能考虑，gRPC还使用了无锁队列这种高级数据结构，不熟悉其原理的读者可能会陷入各种原子操作的细节中。

## grpc_closure 闭包

closure就是闭包的英文名称。简单的理解，闭包函数将创建闭包时的上下文中的变量与自己绑定在一起，将变量的生存期和作用域延长到闭包函数结束。



## 参考

[CSDN：gRPC 源码分析](https://blog.csdn.net/happyanger6/category_9292845.html)