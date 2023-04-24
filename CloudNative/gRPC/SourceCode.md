# 1. 目标

gRPC C++ 源码的质量很高，学习 Google 工程师门优秀的抽象和设计能力。


# 2. 特色


## 2.1. 大量高级抽象结构

gRPC 的源码为了简化异步代码的编写，同时为了更好的代码复用。设计了许多高级的数据结构。如

- grpc_closure
- grpc_closure_scheduler
- ExecCtx
- grpc_combiner
- grpc_completion_queue

## 2.2. 大量设计模式

为了提供跨平台能力，grpc核心代码采用了bridge设计模式，因此可以看到各种vtable,这也为阅读源代码增加了困难。

## 2.3. 异步编程

grpc核心库采用reactor设计模式，如grpc_closure就是为了方便异步编程而设计的数据结构。



## 2.4. 高性能

出于性能考虑，gRPC还使用了无锁队列这种高级数据结构，不熟悉其原理的读者可能会陷入各种原子操作的细节中。

## 2.5. grpc_closure 闭包

closure就是闭包的英文名称。简单的理解，闭包函数将创建闭包时的上下文中的变量与自己绑定在一起，将变量的生存期和作用域延长到闭包函数结束。

## 2.6. 源码

### 2.6.1. 结构

#### 2.6.1.1. src/core/lib/surface/

`grpc_init()` 声明在 include/grpc/grpc.h 文件中，而其定义则是在 src/core/lib/surface/init.cc 文件中。

目录提供了 gRPC 的核心开发 API，并将【核心组件】转成【函数调用】

比如 channel.h 是中的 `grpc_channel` 结构和 `grpc_channel_create_internal()` 函数的声明，在对应的 channel.cc 中有实现。

#### 2.6.1.2. src/core

core 提供了低层次的库，提供给高层次库封装用的。 顶层的 API 在 grpc.h 中声明 安全相关的 在 grpc_security.h 中

- include/grpc/grpc.h 是给 C 语言使用的 API
- include/grpcpp/grpcpp.h 是给 C++ 语言使用的 API

# 3. 参考

[CSDN：gRPC 源码分析](https://blog.csdn.net/happyanger6/category_9292845.html)