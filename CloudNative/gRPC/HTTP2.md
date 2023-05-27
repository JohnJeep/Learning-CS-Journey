<!--
 * @Author: JohnJeep
 * @Date: 2023-05-06 10:28:58
 * @LastEditors: JohnJeep
 * @LastEditTime: 2023-05-27 15:36:37
 * @Description: HTTP2 知识
 * Copyright (c) 2023 by John Jeep, All Rights Reserved. 
-->


## HPACK

在一个 HTTP 请求里面，我们通常在 header 上面携带很多该请求的元信息，用来描述要传输的资源以及它的相关属性。在 HTTP/1.x 时代，我们采用纯文本协议，并且使用 `\r\n`来分隔，如果我们要传输的元数据很多，就会导致 header 非常的庞大。另外，多数时候，在一条连接上面的多数请求，其实 header 差不了多少，譬如我们第一个请求可能 `GET /a.txt`，后面紧接着是 `GET /b.txt`，两个请求唯一的区别就是 URL path 不一样，但我们仍然要将其他所有的 fields 完全发一遍。

HTTP/2 为了结果这个问题，使用了 HPACK。虽然 HPACK 的 [RFC 文档 ](https://httpwg.org/specs/rfc7541.html)看起来比较恐怖，但其实原理非常的简单易懂。

HPACK 提供了一个静态和动态的 table，静态 table 定义了通用的 HTTP header fields，譬如 method，path 等。发送请求的时候，只要指定 field 在静态 table 里面的索引，双方就知道要发送的 field 是什么了。

对于动态 table，初始化为空，如果两边交互之后，发现有新的 field，就添加到动态 table 上面，这样后面的请求就可以跟静态 table 一样，只需要带上相关的 index 就可以了。

同时，为了减少数据传输的大小，使用 Huffman 进行编码。这里就不再详细说明 HPACK 和 Huffman 如何编码了。



## gRPC基于HTTP/2的优缺点

**优点**

- HTTP/2是一个经过实践检验的公开的标准
- 天然支持手机、物联网、浏览器
- 多语言实现容易，每种流行的编程语言都有自己的HTTP/2 Client
- HTTP/2支持Stream和流控
- 基于HTTP/2 在Gateway/Proxy很容易支持
- HTTP/2 安全性有保证
- HTTP/2 鉴权成熟

**缺点**

- RPC 的元数据的传输不够高效
- HTTP/2 里一次 gRPC 调用需要解码两次,一次是HEADERS frame，一次是DATA frame
- HTTP/2 标准本身是只有一个TCP连接，但是实际在 gRPC 里是会有多个TCP连接，使用时需要注意。





# Reference

- 官方RFC文档：https://httpwg.org/specs/rfc7540.html
- 深入了解 gRPC协议：https://cn.pingcap.com/blog/grpc