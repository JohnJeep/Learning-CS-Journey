<!--
 * @Author: JohnJeep
 * @Date: 2023-05-06 15:12:30
 * @LastEditors: JohnJeep
 * @LastEditTime: 2023-05-27 15:37:12
 * @Description: gRPC metadata 学习
 * Copyright (c) 2023 by John Jeep, All Rights Reserved. 
-->


# Metadata

## 概念

在 gRPC 中，metadata 是一些关于请求或响应的元数据信息，以键值对的形式表示，表示特定的RPC调用信息。key 一般是 `strings`，value 一般也是`string`，有时可能是二进制数据。它们用于传递一些非负载（payload）相关的数据，例如认证信息、跟踪 ID、时间戳等等。

metadata 可以在客户端和服务端之间进行传递，可以在请求和响应中包含 metadata，也可以通过 gRPC **拦截器（interceptor）**对其进行修改、添加或删除。metadata 可以在请求和响应的生命周期中的任何时刻进行操作，并且可以自定义和扩展，以满足特定的应用场景。

在 gRPC 的 API 中，metadata 通常使用 gRPC 的 Metadata 类型来表示，这个类提供了一组 API 来方便开发者操作 metadata，例如：

- 添加元数据：metadata.put(key, value)
- 获取元数据：metadata.get(key)
- 删除元数据：metadata.remove(key)

在 gRPC 中，当客户端发起一个请求时，可以通过 Channel 提供的 withMetadata 方法来添加一些 metadata 信息到请求中。这些 metadata 信息将随着请求一起发送到服务端，服务端可以通过拦截器等机制来获取和处理这些 metadata。同样地，服务端也可以在响应中添加 metadata 信息，这些信息将会随着响应一起发送到客户端。

此外，metadata 还可以通过 Channel 提供的 intercept 方法来进行拦截和修改。拦截器是一种类似于中间件的机制，可以在请求和响应的生命周期中对其进行拦截和修改。使用拦截器，可以在请求和响应中添加、删除、修改 metadata 信息，以实现更加灵活和定制化的 metadata 处理。







## 原理



## References

- gRPC 官方解释：https://grpc.io/docs/what-is-grpc/core-concepts/#metadata
- Go 官方库 Metadata API 接口: https://pkg.go.dev/google.golang.org/grpc/metadata#MD.Get
- Chromium metadata Example: https://chromium.googlesource.com/external/github.com/grpc/grpc/+/HEAD/examples/cpp/metadata/
- How to set custom headers on a request 论坛讨论: https://groups.google.com/g/grpc-io/c/C8j3zGtL2-M
- Github HTTP2: https://github.com/grpc/grpc/blob/master/doc/PROTOCOL-HTTP2.md