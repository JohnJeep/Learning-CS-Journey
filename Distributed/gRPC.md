<!--
 * @Author: johnjeep
 * @Date: 2022-11-02 21:34:24
 * @LastEditors: johnjeep
 * @LastEditTime: 2022-11-03 00:17:48
 * @Description: gRPC 用法
 * Copyright (c) 2022 by johnjeep, All Rights Reserved. 
-->
# gRPC



## 服务治理

每个服务启动的时候，会将自身的服务和IP注册到注册中心，其他服务调用的时候，只需要向注册中心申请地址即可。





## RPC(remote produce call)

RPC Architecture

![](figures/remote-call-procedure-working.png)

## gRPC 概念

[gRPC](http://www.grpc.io/) 是一个由 Google 开发的高性能开源通用 RPC 框架。在 gRPC 中，客户端应用可以直接调用其他机器上的服务器应用中的方法，如同调用本地对象一样，从而让您更轻松地创建分布式应用和服务。

使用 gRPC 的主要优势之一是用于生成文档；您可以使用服务配置和 API 接口定义文件来生成 API 的参考文档。



- **Channel** 提供一个与特定 gRPC server 的主机和端口建立的连接。
- **Stub** 就是在 **Channel** 的基础上创建而成的。



## RPC 特点

1. 语言中立，支持多种语言；
2. 基于 IDL 文件定义服务，通过 proto3 工具生成指定语言的数据结构、服务端接口以及客户端 Stub；
3. 通信协议基于标准的 HTTP/2 设计，支持双向流、消息头压缩、单 TCP 的多路复用、服务端推送等特性，这些特性使得 gRPC 在移动端设备上更加省电和节省网络流量；
4. 序列化支持 PB（Protocol Buffer）和 JSON，PB 是一种语言无关的高性能序列化框架，基于 HTTP/2 + PB, 保障了 RPC 调用的高性能。

在一次RPC调用中，负责为客户端代理的节点（gRPC中称之为Stub）会将请求和参数传到服务端，并由Service进行实际的处理，然后将结果返回给Stub，最终返回到客户端中。


## 如何学习 gRPC?

总结一下，学习 RPC 时，我们先要了解其基本原理以及关键的网络通信部分，不要一味依赖现成的框架；之后我们再学习 RPC 的重点和难点，了解 RPC 框架中的治理功能以及集群管理功能等；这个时候你已经很厉害了，但这还不是终点，我们要对 RPC 活学活用，学会提升 RPC 的性能以及它在分布式环境下如何定位问题等等。



## proto 文件

`hello.proto`

```protobuf
// proto buffer 语法版本
syntax = "proto3";

option java_multiple_files = true;
option java_package = "io.grpc.examples.helloworld";
option java_outer_classname = "HelloWorldProto";
option objc_class_prefix = "HLW";

// 包名：用来防止协议消息类型之间发生命名的冲突；C++叫命名空间，Java中叫包名
package helloworld;

// 定义gRPC服务的接口
service Greeter {
  // 远程调用方法；HelloRequest为函数参数，HelloReply为函数返回值
  rpc SayHello (HelloRequest) returns (HelloReply) {}
}

// 定义请求的消息格式和类型 
message HelloRequest {
  string name = 1;  // 唯一字段编号，用于二进制消息格式中识别该字段
}

// 定义响应的消息格式和类型 
message HelloReply {
  string message = 1;
}

```



## gRPC 服务端



## gRPC 客户端



## gRPC 处理流程

当调用 gRPC 服务时，客户端的 gRPC 库会使用 protocol buffers，将 RPC 的请求**编排（masrshal）**为 protocol buffers 格式，然后通过 HTTP/2 进行发送。在服务端，请求会被**解排（unmasrshal）**。而响应也遵循类似的执行流，从服务端发送到客户端。

- 编排：将参数和远程函数打包的过程。
- 解排：解包消息到对应的方法调用的过程。







## gRPC 强大的功能

- 治理功能。比如连接管理、健康检测、负载均衡、优雅启停机、异常重试、业务分组以及熔断限流。

- 集群管理功能。

## gRPC 四种通信模式

1. Unary（一元RPC）

   一元RPC模式也称为简单RPC模式。在该模式中，客户端发送单个请求到服务端，服务端响应单个响应。

   ![Unary Architecture](https://www.polarsparc.com/xhtml/images/grpc-02.png)

2. server-side streaming（服务端流式RPC）

   客户端向服务端发送单个请求，服务端收到客户端的请求后，会处理多个响应，这种多个响应所组成的序列也被称为”流“。

   ![Server Streaming Architecture](https://www.polarsparc.com/xhtml/images/grpc-06.png)

3. client-side-streaming（客户端流式RPC）

   在客户端流 RPC 模式中，客户端会发送多个请求给服务器端，而不再是单个请求  。服务器
   端会发送一个响应给客户端 。 但是，服务器端不一定要等到从客户端接收到所有消息后
   才发送响应。基于这样的逻辑 ，我们可以在接收到流中的一条消息或几条消息之后就发送
   响应，也可以在读取完流中的所有消息之后再发送响应.  

   ![Client Streaming Architecture](https://www.polarsparc.com/xhtml/images/grpc-08.png)

4. Bidirectional Streaming （ 双向流式RPC）

   ![Bidirectional Streaming Architecture](https://www.polarsparc.com/xhtml/images/grpc-09.png)



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
- grpc学习：https://qiankunli.github.io/2020/02/28/grpc.html

可选

- 微服务治理框架(C++版)详细设计：https://github.com/grpc-nebula/grpc-nebula-c/tree/master/docs
- gRPC Load Balancing：https://grpc.io/blog/grpc-load-balancing/