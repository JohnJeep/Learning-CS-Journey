<!--
 * @Author: JohnJeep
 * @Date: 2022-04-08 09:27:21
 * @LastEditors: JohnJeep
 * @LastEditTime: 2023-05-27 15:38:07
 * @Description: Protobuf 知识
 * Copyright (c) 2023 by John Jeep, All Rights Reserved. 
-->


## 概念



## 标识号

在消息体的定义中，每个字段都必须有一个唯一的标识号，标识号是  $0- 2^{29}-1$ 范围内的一个整数。



## 消息类型





## 基本类型



## 枚举类型



## 序列化与反序列化





## 三种流（stream）

- 字节流（byte stream）
- 文件流（file stream）
- 字符串流（string stream） 

protobuf 中定义服务



### v2 和 v3 主要区别

- 删除原始值字段的字段存在逻辑
- 删除 required 字段
- 删除 optional 字段，默认就是
- 删除 default 字段
- 删除扩展特性，新增 Any 类型来替代它
- 删除 unknown 字段的支持
- 新增 [JSON Mapping](https://developers.google.com/protocol-buffers/docs/proto3#json)
- 新增 Map 类型的支持
- 修复 enum 的 unknown 类型
- repeated 默认使用 packed 编码
- 引入了新的语言实现（C＃，JavaScript，Ruby，Objective-C）

以上是日常涉及的常见功能，如果还想详细了解可阅读 [Protobuf Version 3.0.0](https://github.com/protocolbuffers/protobuf/releases?after=v3.2.1)



## Protobuf 适用场景

思考方向：使用这个协议会产生什么影响。

1. 网络带宽
2. 吞吐量
3. 响应速度
4. 传输效率
5. 存储成本



### 处理大数据包局限

1. 内存占用：在反序列化大型 Protobuf 数据包时，需要将整个数据包加载到内存中。这可能导致内存占用较高，因此在处理非常大的数据包时，可能需要考虑分块处理或其他优化策略。
2. 消息大小限制：Protobuf 有一个默认的消息大小限制（默认为 64MB），超过该限制的数据包将无法正常处理。可以通过调整配置或使用流式处理来解决此问题，但需要注意潜在的影响。



# References

- Google 官网教程：https://developers.google.com/protocol-buffers
- Github 地址：https://github.com/protocolbuffers/protobuf
- Protocol Buffers V3中文语法指南：https://www.liwenzhou.com/posts/Go/Protobuf3-language-guide-zh/
- Github Google APIs: https://github.com/googleapis/googleapis/tree/master
- Google APIs: https://google.aip.dev/general



思考：编译器中的类型是如何在内存中存储的，不同的类型是怎样区分的？