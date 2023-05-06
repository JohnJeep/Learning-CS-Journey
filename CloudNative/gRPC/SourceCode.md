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



### Status

在gRPC C++ API中，Status是一个封装了错误代码和错误消息的类。它在gRPC中的各个组件中都有应用，例如客户端、服务器和中间件。

当客户端或服务器调用远程过程调用（RPC）时，它们可能会遇到各种状态，如网络错误、超时、未授权访问等。在这些情况下，gRPC C++ API将向调用方返回一个Status实例，该实例包含一个错误代码和一个可选的错误消息。

错误代码在`grpc::Status::Code`枚举中定义，并包含了gRPC支持的所有错误类型。如果错误消息不为空，它将包含有关错误的详细信息。

通过检查返回的Status实例，程序员可以方便地检查处理gRPC调用时的错误情况，并根据需要采取相关的行动。在一些情况下，调用方可能会利用Status实例的错误信息，向其他代码、日志系统或应用程序用户通知错误发生的情况，以便执行更多的错误处理、调试和故障排除工作。



## GoAway 机制


gRPC GoAway 是 gRPC 协议中的一种机制，用于在一个 gRPC 服务器从其客户端终止连接时通知客户端。当服务器决定关闭与一个或多个客户端的连接时，它会向这些客户端发送一个 GoAway 帧，以便它们能够进行一些清理操作。

GoAway 帧包含一个最后有效流标识符 (Last-Stream-ID) 和一个可选的错误代码 (Error Code)，指示服务器关闭连接的原因。当客户端收到 GoAway 帧时，它应该停止发送新的请求，并关闭与服务器的连接，同时尽可能处理之前已发送的响应。客户端也可以查看 GoAway 帧的 Last-Stream-ID，以确定是否需要重新发起某些请求。

gRPC GoAway 机制是一种可靠的方法，用于管理长连接和复杂的通信场景，例如负载均衡和故障转移。它可以帮助确保客户端和服务器之间的连接得到有效管理，从而提高系统的可靠性和性能。



# gRPC epoll architecture



- https://github.com/grpc/grpc/blob/master/doc/core/epoll-polling-engine.md
- https://yiakwy.github.io/blog/2017/10/01/gRPC-C-CORE



# Visualizing gRPC Language Stacks

https://grpc.io/blog/grpc-stacks/

gRPC: Under the Hood: https://www.oreilly.com/library/view/grpc-up-and/9781492058328/ch04.html



# 3. References

- CSDN：gRPC 源码分析: https://blog.csdn.net/happyanger6/category_9292845.html
- RPC原理以及GRPC详解: https://www.cnblogs.com/awesomeHai/p/liuhai.html