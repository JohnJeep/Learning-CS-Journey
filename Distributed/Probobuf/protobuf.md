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





# 参考

- Google 官网教程：https://developers.google.com/protocol-buffers
- Github 地址：https://github.com/protocolbuffers/protobuf
- Protocol Buffers V3中文语法指南：https://www.liwenzhou.com/posts/Go/Protobuf3-language-guide-zh/



思考：编译器中的类型是如何在内存中存储的，不同的类型是怎样区分的？