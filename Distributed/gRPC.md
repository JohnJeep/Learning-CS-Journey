<!--
 * @Author: johnjeep
 * @Date: 2022-11-02 21:34:24
 * @LastEditors: johnjeep
 * @LastEditTime: 2022-11-03 00:17:48
 * @Description: gRPC 用法
 * Copyright (c) 2022 by johnjeep, All Rights Reserved. 
-->
# gRPC



## RPC(remote produce call)

RPC Architecture

![](figures/remote-call-procedure-working.png)

## gRPC 概念

[gRPC](http://www.grpc.io/) 是一个由 Google 开发的高性能开源通用 RPC 框架。在 gRPC 中，客户端应用可以直接调用其他机器上的服务器应用中的方法，如同调用本地对象一样，从而让您更轻松地创建分布式应用和服务。

使用 gRPC 的主要优势之一是用于生成文档；您可以使用服务配置和 API 接口定义文件来生成 API 的参考文档。



- **Channel** 提供一个与特定 gRPC server 的主机和端口建立的连接。
- **Stub** 就是在 **Channel** 的基础上创建而成的。





## gRPC 四种通信模式

1. Unary
2. server-side streaming
3. client-side-streaming
4. Bidirectional Streaming 



## 异步API

C++ gRPC 异步的操作使用 CompletionQueue 来实现的。任何异步操作的完成都是完成队列中的一个事件。



调用基本的流程：

- 绑定一个 `CompletionQueue` 到一个 RPC 调用
- 利用唯一的 `void*` Tag 进行读写
- 调用 `CompletionQueue::Next()` 等待操作完成，完成后通过唯一的 Tag 来判断对应什么请求/返回进行后续操作





## 源码

### 结构

#### src/core/lib/surface/

`grpc_init()` 声明在 include/grpc/grpc.h 文件中，而其定义则是在 src/core/lib/surface/init.cc 文件中。

目录提供了 gRPC 的核心开发 API，并将【核心组件】转成【函数调用】

比如 channel.h 是中的 `grpc_channel` 结构和 `grpc_channel_create_internal()` 函数的声明，在对应的 channel.cc 中有实现。****

#### src/core

core 提供了低层次的库，提供给高层次库封装用的。 顶层的 API 在 grpc.h 中声明 安全相关的 在 grpc_security.h 中

- include/grpc/grpc.h 是给 C 语言使用的 API
- include/grpcpp/grpcpp.h 是给 C++ 语言使用的 API





## Reference

- [What is gRPC? Protocol Buffers, Streaming, and Architecture Explained](https://www.freecodecamp.org/news/what-is-grpc-protocol-buffers-stream-architecture/)

- [Introduction to gRPC Part1](https://www.polarsparc.com/xhtml/gRPC-1.html)
- [Introduction to gRPC Part2](https://www.polarsparc.com/xhtml/gRPC-2.html)
- [Introduction to gRPC Part3](https://www.polarsparc.com/xhtml/gRPC-3.html)
- [Introduction to gRPC Part4](https://www.polarsparc.com/xhtml/gRPC-4.html)
- [gRPC 代码使用的 C/C++ 技巧](https://panzhongxian.cn/cn/2021/09/grpc-cpp-tricks/)
- gRPC 博客归档：https://panzhongxian.cn/tags/grpc/
- C++ gRPC 异步 API 实例与优势：https://juejin.cn/post/6998554231837818917

可选

- 微服务治理框架(C++版)详细设计：https://github.com/grpc-nebula/grpc-nebula-c/tree/master/docs
- gRPC Load Balancing：https://grpc.io/blog/grpc-load-balancing/