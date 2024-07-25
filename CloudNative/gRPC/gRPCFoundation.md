 <!-- 
 * @Author: JohnJeep
 * @Date: 2022-11-02 21:34:24
 * @LastEditors: JohnJeep
 * @LastEditTime: 2024-03-31 10:05:52
 * @Description: gRPC 用法
 * Copyright (c) 2022 by johnjeep, All Rights Reserved.  
-->

<!-- TOC -->

- [1. RPC(remote produce call)](#1-rpcremote-produce-call)
  - [1.1. 什么是 RPC](#11-什么是-rpc)
  - [RPC 局限性](#rpc-局限性)
- [2. gRPC](#2-grpc)
  - [2.1. 如何学习 gRPC?](#21-如何学习-grpc)
  - [2.2. 概念](#22-概念)
    - [2.2.1. Channel](#221-channel)
    - [2.2.2. Stub](#222-stub)
    - [2.2.3. Metadata](#223-metadata)
  - [1.2. gRPC 特点](#12-grpc-特点)
  - [1.3. Service definition](#13-service-definition)
    - [Request](#request)
    - [Response](#response)
    - [1.3.1. 流的结束](#131-流的结束)
  - [1.2. proto 文件](#12-proto-文件)
  - [1.3. gRPC 服务端](#13-grpc-服务端)
  - [1.4. gRPC 客户端](#14-grpc-客户端)
    - [1.4.1. 同步（Synchronous）](#141-同步synchronous)
  - [1.2. 同步RPC缺点](#12-同步rpc缺点)
    - [1.2.1. 异步（Asynchronous）](#121-异步asynchronous)
      - [1.2.1.1. 实现 C++ gRPC 异步双向流注意事项](#1211-实现-c-grpc-异步双向流注意事项)
      - [1.2.1.2. API接口](#1212-api接口)
      - [1.2.1.3. 多线程](#1213-多线程)
  - [1.2. gRPC 处理流程](#12-grpc-处理流程)
  - [](#)
  - [1.4. gRPC 强大的功能](#14-grpc-强大的功能)
    - [1.4.1. 服务治理](#141-服务治理)
    - [保活机制](#保活机制)
  - [熔断机制/限流机制](#熔断机制限流机制)
    - [负载均衡](#负载均衡)
    - [channel 复用](#channel-复用)
    - [flow control（流量控制）](#flow-control流量控制)
- [2. Performance](#2-performance)
- [3. References](#3-references)

<!-- /TOC -->


# 1. RPC(remote produce call)

## 1.1. 什么是 RPC

RPC简称远程过程调用，是一个用于构建基于Client和Server分布式应用程序的技术。目前业界已经有了很多的框架能够用来构建基于RPC的分布式应用，例如SpringBoot，Dubbo和gRPC。

RPC 标准最早是由Bruce Jay Nelson 写的论文 [Implementing Remote Procedure Calls](http://www.cs.cmu.edu/~dga/15-712/F07/papers/birrell842.pdf)中提出的，后期的所有的RPC框架都是在这个标准模式的基础上构建出来的。

![](../figures/remote-call-procedure-working.png)

具体的执行过程就是下面这个样子

- 客户端发起一个远程调用,它实际上是调用本地的Client Stub
- Client Stub 将接受到的参数进行按照约定的协议规范进行编码，并封装到即将发送的Message中。
- Client Stub 将消息发送给RPC Runtime，然后通过网络将包 发送给Server端
- 服务器端的 RPCRuntime 收到请求后，交给提供方 Stub 进行解码，然后调用服务端的方法， 服务端执行方法，返回结果
- 服务端的处理结果 同样再经过Server Stub 打包，然后传递给RPC Runtime。
- 服务端的RPC Runtime再把数据通过网络发送给Client端。
- Client 端接收到消息，然后进行 Unpack 处理。


RPC 实现例子：
![Alt text](../figures/grpc-call.png)


## RPC 局限性

- 本地函数调用的结果是可预测的，而 RPC 需要经过网络传输，数据在中途可能因为各种原因丢失。
- RPC 调用有可能超时，编写程序时需要考虑该情况。
- 重试一个失败的 RPC 调用有可能造成数据重复，需要考虑幂等。
- 由于传输数据时需要序列化和反序列化，RPC 在传输复杂对象时会不太方便。



# 2. gRPC

## 2.1. 如何学习 gRPC?

总结一下，学习 RPC 时，我们先要了解其基本原理以及关键的网络通信部分，不要一味依赖现成的框架；之后我们再学习 RPC 的重点和难点，了解 RPC 框架中的治理功能以及集群管理功能等；这个时候你已经很厉害了，但这还不是终点，我们要对 RPC 活学活用，学会提升 RPC 的性能以及它在分布式环境下如何定位问题等等。

1. 掌握 grpc 基础理论
2. 会用 grpc API 去做一些简单的 server、client
3. 掌握 grpc 功能
4. 阅读源码
5. 深刻理解 grpc 功能，成为一个高手



## 2.2. 概念

[gRPC](http://www.grpc.io/) 是一个由 Google 开发的高性能开源通用 RPC 框架。在 gRPC 中，客户端应用可以直接调用其他机器上的服务器应用中的方法，如同调用本地对象一样，从而让您更轻松地创建分布式应用和服务。

使用 gRPC 的主要优势之一是用于生成文档；您可以使用服务配置和 API 接口定义文件来生成 API 的参考文档。

### 2.2.1. Channel

在 gRPC 中，Channel 是客户端与服务端之间通信的通道。当客户端需要与服务端进行通信时，它需要创建一个 Channel 对象，通过这个对象来发送请求和接收响应。

Channel 是一个连接池，它会缓存与服务端的连接以提高请求的效率。当客户端发送一个请求时，它会从连接池中获取一个可用的连接，如果连接池中没有可用连接，则会创建一个新的连接。

除了发送请求和接收响应外，Channel 还可以提供一些其他的功能，例如**拦截器、事件监听、性能指标**等等。

- 拦截器可以用来拦截请求和响应，并对其进行处理，例如添加认证信息、日志记录等等。
- 事件监听器可以监听 Channel 中发生的事件，例如连接建立、连接断开、请求发送等等。
- 性能指标可以用来监控 Channel 中请求的成功率、延迟等指标，帮助开发者进行性能优化。



> - Clients 可以指定 channel 的参数，去修改gRPC 的默认行为，例如：开启或关闭消息压缩的开关。
> - 一个 channel 是具有状态的，包括 `idle` 和 `connected`。



### 2.2.2. Stub 

在 gRPC 中，Stub 是客户端使用的一个代理类，它实现了客户端与服务端之间通信的逻辑。通过 Stub 对象，客户端可以调用远程服务提供的 RPC 方法，并将请求发送到服务端进行处理，同时接收服务端返回的响应结果。

在 gRPC 中，每个服务都有一个对应的 Stub 类，该类包含了该服务定义的所有 RPC 方法，客户端可以通过该 Stub 对象来调用这些方法。在 gRPC 中，Stub 类是由 Protocol Buffers 编译器自动生成的，根据服务定义的 `.proto` 文件，编译器会生成一个对应的 Stub 类，并将其与客户端代码一起编译。

> 官方解释：*On the client side, the client has a local object known as stub (for some languages, the preferred term is client) that implements the same methods as the service. The client can then just call those methods on the local object, and the methods wrap the parameters for the call in the appropriate protocol buffer message type, send the requests to the server, and return the server’s protocol buffer responses.*

### 2.2.3. Metadata





## 1.2. gRPC 特点

1. 语言中立，支持多种语言；
2. 基于 IDL 文件定义服务，通过 proto3 工具生成指定语言的数据结构、服务端接口以及客户端 Stub；
3. 通信协议基于标准的 HTTP/2 设计，支持双向流、消息头压缩、单 TCP 的多路复用、服务端推送等特性，这些特性使得 gRPC 在移动端设备上更加省电和节省网络流量；
4. 序列化支持 PB（Protocol Buffer）和 JSON，PB 是一种语言无关的高性能序列化框架，基于 HTTP/2 + PB, 保障了 RPC 调用的高性能。

在一次RPC调用中，负责为客户端代理的节点（gRPC中称之为Stub）会将请求和参数传到服务端，并由Service进行实际的处理，然后将结果返回给Stub，最终返回到客户端中。



## 1.3. Service definition

gRPC 有 4  种请求和响应模式。

gRPC 是 Google 基于 HTTP/2 以及 protobuf 的，要了解 gRPC 协议，只需要知道 gRPC 是如何在 HTTP/2 上面传输就可以了。

gRPC 通常有四种模式，unary，client streaming，server streaming 以及 bidirectional streaming，对于底层 HTTP/2 来说，它们都是 stream，并且仍然是一套 request + response 模型。

### Request

gRPC 的 request 通常包含 Request-Headers, 0 或者多个 Length-Prefixed-Message 以及 EOS。

Request-Headers 直接使用的 HTTP/2 headers，在 HEADERS 和 CONTINUATION frame 里面派发。定义的 header 主要有 Call-Definition 以及 Custom-Metadata。Call-Definition 里面包括 Method（其实就是用的 HTTP/2 的 POST），Content-Type 等。而 Custom-Metadata 则是应用层自定义的任意 key-value，key 不建议使用 `grpc-`开头，因为这是为 gRPC 后续自己保留的。

Length-Prefixed-Message 主要在 DATA frame 里面派发，它有一个 Compressed flag 用来表示改 message 是否压缩，如果为 1，表示该 message 采用了压缩，而压缩算法定义在 header 里面的 Message-Encoding 里面。然后后面跟着四字节的 message length 以及实际的 message。

EOS（end-of-stream） 会在最后的 DATA frame 里面带上了 `END_STREAM`这个 flag。用来表示 stream 不会在发送任何数据，可以关闭了。

### Response

Response 主要包含 Response-Headers，0 或者多个 Length-Prefixed-Message 以及 Trailers。如果遇到了错误，也可以直接返回 Trailers-Only。

Response-Headers 主要包括 HTTP-Status，Content-Type 以及 Custom-Metadata 等。Trailers-Only 也有 HTTP-Status ，Content-Type 和 Trailers。Trailers 包括了 Status 以及 0 或者多个 Custom-Metadata。

HTTP-Status 就是我们通常的 HTTP 200，301，400 这些，很通用就不再解释。Status 也就是 gRPC 的 status， 而 Status-Message 则是 gRPC 的 message。Status-Message 采用了 Percent-Encoded 的编码方式，具体参考 [这里 ](https://tools.ietf.org/html/rfc3986#section-2.1)。

如果在最后收到的 HEADERS frame 里面，带上了 Trailers，并且有 `END_STREAM`这个 flag，那么就意味着 response 的 EOS。





1. Unary（一元 RPC）

   一元RPC模式也称为简单RPC模式。客户端发送单个请求到服务端，等待服务端的响应。

   ![Unary Architecture](https://www.polarsparc.com/xhtml/images/grpc-02.png)

   缺点：

   - 数据包过大造成的瞬时压力

   - 接收数据包时，需要所有数据包都接受成功且正确后，才能够回调响应，进行业务处理（无法客户端边发送，服务端边处理）

2. server-side streaming（服务端流式RPC）

   当数据量大或者需要不断传输数据时候，我们应该使用流式RPC，它允许我们边处理边传输数据。

   服务端流式RPC：客户端向服务端发送单个请求，服务端收到客户端的请求后，处理多个响应，这种多个响应所组成的序列被称为”流（stream）“。客户端读来自服务端返回的 stream，直到没有消息为止。

   ![Server Streaming Architecture](https://www.polarsparc.com/xhtml/images/grpc-06.png)

   

   

3. client-side-streaming（客户端流式RPC）

   在客户端流 RPC 模式中，客户端发送（write）多个请求给服务器端，而不再是单个请求，一旦客户端完成了发送消息，它等服务端去读（read）完一个序列的消息，由服务端返回一个响应 （response）。

   ![Client Streaming Architecture](https://www.polarsparc.com/xhtml/images/grpc-08.png)

   

4. Bidirectional Streaming （ 双向流式RPC）

   双向流式RPC 模式：client 发送一定消息序列的请求（request）给 server，而 server 给 client 回的响应也是一定消息序列的响应。双方使用读写流（a read-write stream）去发送一个消息序列，**两个流独立操作**，双方可以同时发送和同时接收。

   ![Bidirectional Streaming Architecture](https://www.polarsparc.com/xhtml/images/grpc-09.png)

   注意点：

   - 避免 **race condition** 或 **deadlocks**。
   

在 gRPC 双向流中，每个传输的消息都有一个头部和一个消息体。其中，消息头部包括了消息的元数据，如消息类型、编解码信息、消息 ID 等；消息体则包含了具体的消息内容。这些信息都是通过 Protocol Buffers 进行编解码的。





### 1.3.1. 流的结束

- Client 发送流：通过 `Writer->WritesDone()` 结束流
- Server 发送流：通过结束 rpc 调用并返回状态码`status code`的方式来结束流
- 读取流：通过 `Reader->Read()` 返回的 bool 型状态，来判断流是否结束



## 1.2. proto 文件

gRPC 默认采用 protocol buffers 数据传输格式。protocol buffers 是google开发的一种能够将结构数据序列化的数据描述语言。

使用protocol buffers的第一步是要在扩展名为.proto的proto文件中定义序列化的数据的结构。 Protocol buffer 数据会被结构化成一个 `message`,而这个 `message` 其实就是一条包含了一些属性（name-value对）的记录。

hello.proto

```protobuf
// proto buffer 语法版本
syntax = "proto3";

// 只对Java语言有效
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

生成 `pb.cc`、`pb.h` 、`grpc.cc`、`grpc.pb.h` 文件。

```shell
// 指定路径下生成 cpp 文件到指定的路径
protoc --proto_path=. --plugin=protoc-gen-grpc=`which grpc_cpp_plugin` --cpp_out=/workspaces/xiot-platform/xiot-platform/iot/server/IOT-ProductServer/proto ./industrial_gateway.proto ./google/protobuf/struct.proto ./google/api/field_behavior.proto

// 指定路径下生成 grpc 文件到指定的路径
 protoc --proto_path=. --plugin=protoc-gen-grpc=`which grpc_cpp_plugin` --grpc_out=/workspaces/xiot-platform/xiot-platform/iot/server/IOT-ProductServer/proto ./industrial_gateway.proto ./google/protobuf/struct.proto ./google/api/field_behavior.proto 
```



## 1.3. gRPC 服务端



## 1.4. gRPC 客户端

gRPC 支持两种类型的 client stub。

### 1.4.1. 同步（Synchronous）

同步rpc调用调用完后，client 不立即=返回，而是等（wait）server 返回的响应。



## 1.2. 同步RPC缺点

1. 线程利用率低：线程资源是系统中非常重要的资源，在一个进程中线程总数是有限制的，提升线程使用率就能够有效提升系统的吞吐量，在同步 RPC 调用中，如果服务端没有返回响应，客户端业务线程就会一直阻塞，无法处理其它业务消息。
2. 纠结的超时时间：RPC 调用的超时时间配置是个比较棘手的问题。如果配置的过大，一旦服务端返回响应慢，就容易把客户端挂死。如果配置的过小，则超时失败率会增加。即便参考测试环境的平均和最大时延来设置，由于生产环境数据、硬件等与测试环境的差异，也很难一次设置的比较合理。另外，考虑到客户端流量的变化、服务端依赖的数据库、缓存、第三方系统等的性能波动，这都会导致服务调用时延发生变化，因此，依靠超时时间来保障系统的可靠性，难度很大。
3. 雪崩效应：在一个同步调用链中，只要下游某个服务返回响应慢，会导致故障沿着调用链向上游蔓延，最终把整个系统都拖垮，引起雪崩



### 1.2.1. 异步（Asynchronous）

概念：client 采用非阻塞式（non-blocking）的去调用 server 端返回的响应。异步API 会阻塞线程，直到一个接收或发送一个消息（message）。

**异步通信要考虑并行和并发。**

C++ gRPC 异步的操作是采用 **CompletionQueue** 来实现的。**CompletionQueue** 是一个 `event queue`。任何异步操作的完成都是完成队列中的一个事件。

异步调用是从 CompletionQueue 中取得响应返回的结果。

- 在异步客户端中，通过`gRPC` `stub` 的异步方法调用，获取`ClientAsyncResponseReader`的实例。
- 在异步客户端中，`ClientAsyncResponseReader` 的Finish方法向 `CompletionQueue`注册了响应消息处理器和响应消息体的存储容器。
- 当服务器响应消息到来时，响应消息体被填充到注册的容器中，而响应消息处理器则被push到`CompletionQueue`中。
- 从`CompletionQueue`中获取到响应消息处理器，对响应消息进行处理。

调用基本的流程：

- 绑定一个 `CompletionQueue` 到一个 RPC 调用
- 利用唯一的 `void*` Tag 进行读写
- 调用 `CompletionQueue::Next()` 等待操作完成，完成后通过唯一的 Tag 来判断对应什么请求/返回进行后续操作。tag：根据`CompletionQueue`返回的 tag 知道是哪个对象产生了事件。

#### 1.2.1.1. 实现 C++ gRPC 异步双向流注意事项

1. 使用异步 API：使用 C++ gRPC 提供的异步 API，如 `AsyncReaderWriter`，而不是同步 API。异步 API 能够在单线程内处理多个客户端请求，提高程序的并发性能。
2. 线程安全：C++ gRPC 是线程安全的，但需要保证在使用 gRPC API 时，线程安全性不受影响。需要注意避免多个线程同时访问同一资源导致的竞态条件问题。
3. 序列化和反序列化：需要将请求和响应数据序列化和反序列化。C++ gRPC 默认使用 Protocol Buffers 作为序列化格式，需要定义好请求和响应消息的格式。
4. 错误处理：在处理异步双向流时，需要注意错误处理。例如，当客户端断开连接时，需要正确处理该事件，避免导致程序异常退出。
5. 流控制：需要合理地使用流控制机制，避免客户端和服务端之间的数据流量过大导致程序崩溃或者性能下降。
6. 代码清晰度：异步编程需要编写更复杂的代码，需要保证代码结构清晰易读，便于维护和修改。



#### 1.2.1.2. API接口

- `StartCall`：开始异步RPC调用。

- `Finish`：等待服务器的响应。

- `ShutDown()` 作用

  在gRPC中，ShutDown()是一个方法，用于安全地停止gRPC服务器或客户端的所有RPC处理。它可以使gRPC平滑地退出，并等待任何未完成的RPC处理完成后关闭gRPC，同时确保不会丢失任何信息。
  
  具体来说，对于服务器：
  
  当服务器调用Shutdown()方法时，它将不再处理新的RPC请求，并且直到所有未完成的RPC请求都被处理完毕后才关闭服务器端口, 这样做的好处是，可以确保所有客户端完成所有最终的数据交换。
  
  应用程序可以调用Shutdown方法以便更改服务配置，例如重新加载证书、更改端口等。
  
  也可以设置超时选项，如果所有RPC请求在超时之前没能完成，那么服务器会强制经由Shutdown清理所有未完成的请求，如果不指定超时选项，服务器将一直等待正在处理的请求，即使超过了Graceful Shutdown Deadlin，然后关闭端口。
  
  对于客户端：
  
  当客户端调用Shutdown()方法时，它将使所有未完成的RPC请求立即终止，并且将不会处理新的RPC请求。
  
  客户端在调用Shutdown()方法后仍可创建新的Channels和Stubs实例，但这些实例将在未来的任何RPC处理期间忽略它们。
  
  综上，gRPC中的ShutDown()方法对于确保RPC请求恰当、安全地完成很重要。在服务器和客户端中，ShutDown()方法被用于清理并停止所有正在运行的RPC处理。如果不通过Shutdown()来关闭gRPC，同时存在未完成的RPC请求时，会存在内存泄漏和丢失信息的风险。
  
- `Next`：一旦服务器的响应到达，调用`cq.Next`方法来获取响应，并检查状态是否OK。

- `PrepareAsyncYourMethod`：准备一个异步的RPC调用。

- `grpc::WriteOptions` 

  - grpc::WriteOptions 是构建 gRPC 的时候使用的选项之一，它用于控制服务器在响应请求时的行为。在 gRPC C++ 中，可以使用 `grpc::WriteOptions` 类来设置写入选项。其中，`writeOptions.set_last_message(true)` 方法用于设置当前消息是否为 Bidirectional Stream 中的最后一个消息。当设置为 true 时，表示当前消息为最后一个消息，否则表示当前消息为中间消息。
  
  - `grpc::WriteOptions` 主要有以下作用：
    - 用于控制服务器的流控制：当客户端向服务器发送数据时，服务器可以使用 WriteOptions 来控制数据的流量，例如限制数据的大小、速度等。
    - 用于控制服务器的压缩：当客户端向服务器发送数据时，服务器可以使用 WriteOptions 来控制数据的压缩方式，例如使用 Gzip 压缩或其它压缩方式。
    - 用于控制服务器的 Acknowledgement：当客户端发送数据给服务器时，服务器可以使用 WriteOptions 来控制何时发送 Acknowledgement，以及 Acknowledgement 的格式等。
  
  

```
为了防止阻塞，Read，Write和Finish都将从CompletionQueue上的一个Event标记返回，以表明每个操作何时完成。在一个while循环中持续等待，直到一些事件被触发。如果事件是状态事件，那么while循环将退出，否则事件将处理并继续循环等待下一个事件。
```



#### 1.2.1.3. 多线程

对于如何在多线程中使用异步 RPC API 完成队列，官方的的文档说明是：Right now, the best performance trade-off is having numcpu's threads and one completion queue per thread.

当前，最好的权衡性能的方法是使用创建 cpu 个数的线程数，并在每个线程中都使用一个完成队列。

在高并发和效率方面，使用CompletionQueue可充分利用现代多核处理器。此外，gRPC库本身已经进行了优化以提高性能。最后，您还可以使用**流量控制**来限制客户端和服务器之间的通信速度。



## 1.2. gRPC 处理流程

当调用 gRPC 服务时，客户端的 gRPC 库会使用 protocol buffers，将 RPC 的请求**编排（masrshal）**为 protocol buffers 格式，然后通过 HTTP/2 进行发送。在服务端，请求会被**解排（unmasrshal）**。而响应也遵循类似的执行流，从服务端发送到客户端。

- 编排：将参数和远程函数打包的过程。
- 解排：解包消息到对应的方法调用的过程。

## 

## 1.4. gRPC 强大的功能

- 治理功能。比如连接管理、健康检测、负载均衡、优雅启停机、异常重试、业务分组以及熔断限流。
- 集群管理功能。

### 1.4.1. 服务治理

每个服务启动的时候，会将自身的服务和IP注册到注册中心，其他服务调用的时候，只需要向注册中心申请地址即可。

### 保活机制

## 熔断机制/限流机制

### 负载均衡

### channel 复用

### flow control（流量控制）

HTTP/2 也支持流控，如果 sender 端发送数据太快，receiver 端可能因为太忙，或者压力太大，或者只想给特定的 stream 分配资源，receiver 端就可能不想处理这些数据。譬如，如果 client 给 server 请求了一个视频，但这时候用户暂停观看了，client 就可能告诉 server 别在发送数据了。

虽然 TCP 也有 flow control，但它仅仅只对一个连接有效果。HTTP/2 在一条连接上面会有多个 streams，有时候，我们仅仅只想对一些 stream 进行控制，所以 HTTP/2 单独提供了流控机制。Flow control 有如下特性：

- Flow control 是单向的。Receiver 可以选择给 stream 或者整个连接设置 window size。
- Flow control 是基于信任的。Receiver 只是会给 sender 建议它的初始连接和 stream 的 flow control window size。
- Flow control 不可能被禁止掉。当 HTTP/2 连接建立起来之后，client 和 server 会交换 SETTINGS frames，用来设置 flow control window size。
- Flow control 是 hop-by-hop，并不是 end-to-end 的，也就是我们可以用一个中间人来进行 flow control。













# 2. Performance





# 3. References

- gRPC 英文官方文档：https://grpc.io/
- Github仓库解释gRPC设计的原理：https://github.com/grpc/proposal
- [gRPC 中文文档](http://doc.oschina.net/grpc?t=61534)：与英文版本不同步，不是最新版本。
- [What is gRPC? Protocol Buffers, Streaming, and Architecture Explained](https://www.freecodecamp.org/news/what-is-grpc-protocol-buffers-stream-architecture/)
- [Introduction to gRPC Part1](https://www.polarsparc.com/xhtml/gRPC-1.html)
- [Introduction to gRPC Part2](https://www.polarsparc.com/xhtml/gRPC-2.html)
- [Introduction to gRPC Part3](https://www.polarsparc.com/xhtml/gRPC-3.html)
- [Introduction to gRPC Part4](https://www.polarsparc.com/xhtml/gRPC-4.html)
- [gRPC 代码使用的 C/C++ 技巧](https://panzhongxian.cn/cn/2021/09/grpc-cpp-tricks/)
- gRPC 博客归档：https://panzhongxian.cn/tags/grpc/
- C++ gRPC 异步 API 实例与优势
  - https://juejin.cn/post/6998554231837818917
  - https://stackoverflow.com/questions/68767309/difference-between-sync-and-async-grpc
  - https://stackoverflow.com/questions/64639004/grpc-c-async-helloworld-client-example-doesnt-do-anything-asynchronously
- **grpc学习**：https://qiankunli.github.io/2020/02/28/grpc.html
- gRPC博客学习归档：https://www.cnblogs.com/FireworksEasyCool/category/1693727.html
- [grpc使用记录(三)简单异步服务实例](https://www.cnblogs.com/oloroso/p/11345266.html)：C++ 实现
- [聊一下 gRPC 的 C++ 异步编程](https://www.luozhiyun.com/archives/671)
- [C++ gRPC 异步 API 实例与优势](https://blog.miigon.net/posts/cn-so-difference-between-sync-and-async-grpc/)
- [Lessons learnt from writing asynchronous streaming gRPC services in C++](https://www.gresearch.co.uk/blog/article/lessons-learnt-from-writing-asynchronous-streaming-grpc-services-in-c/) ：grpc 异步服务端流模式例子。
- **Github awesome-grpc:** https://github.com/grpc-ecosystem/awesome-grpc
- https://www.selinux.tech/golang/grpc/what-grpc



可选

- 微服务治理框架(C++版)详细设计：https://github.com/grpc-nebula/grpc-nebula-c/tree/master/docs
- gRPC Load Balancing：https://grpc.io/blog/grpc-load-balancing/

gRPC Issues

- C++ Asynchronous Streaming RPC example #10013: https://github.com/grpc/grpc/issues/10013
- C++ Async bidi streaming sample #8934: https://github.com/grpc/grpc/pull/8934
- Provide a simple event-processing loop for C++ async API #7352: https://github.com/grpc/grpc/issues/7352

其它应对方案

- Tradias/[asio-grpc: https://github.com/Tradias/asio-grpc
- agrpc: https://github.com/npuichigo/agrpc



- 思考gRPC ：为什么是HTTP/2：https://blog.csdn.net/hengyunabc/article/details/81120904

- 从实践到原理，带你参透 gRPC：https://segmentfault.com/a/1190000019608421 文章写的很详细，值得参考。

