<!--
 * @Author: JohnJeep
 * @Date: 2020-05-13 10:25:24
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-02-13 10:01:58
 * @Description: jsoncpp learning
 * Copyright (c) 2025 by John Jeep, All Rights Reserved. 
-->

# 1. Introduction

JsonCpp 是一个序列化反序列 JSON 格式的开源 C++库，被 C++程序广泛使用（包括 Chromium 项目）。JsonCpp
还有一个重要特性是其支持在 JSON 格式内注释，这对于使用 JSON
格式作为配置文件很有意义，可以给配置添加注释说明其用途。


# 2. Usaging

JsonCpp 三个核心类 Reader、FastWriter、Value 基本可以满足项目对 JSON 构造解析的要求。

- **类 Reader**，用来将一个 JSON 文件或 JSON 格式的字符串解析成 Value 对象。
  parse()接口第一个参数为 JSON 格式字符串，第二个参数是解析后 Value 对象，如果 JSON 格式正确将解析成功。
- **类 FastWriter**，用来将一个 Value 对象格式化为 JSON 格式的字符串。
  write()接口的参数是一个 Value 对象，返回值为 JSON 格式的字符串。
- **类 Value**，是 JsonCpp 库最为重要的类，它代表 JSON 格式字符串在内存中的状态，修改 JSON 格式字符串需先修改其 Value
  对象，然后序列化输出，其提供四类接口：
  - 第一， 判断类型，接口名字为 isXXX()，其中 XXX 为类型，包括
    Bool、Int、Int64、UInt、UInt64、Double、String、Array、Object，与 JSON 格式的类型是对应的，isNull
    用来判断是否为空。
  - 第二， 取值，接口名字为 asXXX()，其中 XXX 与判断类型的接口一样，取值前务必先确保类型是对的，否则会抛出逻辑错误的异
    常。类型为 Array 的时候，size()接口获取 Array 的个数，然后遍历获取 Array
    每个值（注意遍历时下标从 0 开始）。类型为 Object 的时候，isMember()接口用来判断对象是否有某个 key，访问该 key
    前务必先确保有该 key，否则会抛出逻辑错误的异常，访问某个 key
    时使用操作符[]，参数为 key 值，有时候不知道对象都有哪些 key，就得先调用 getMemberNames()接口获取 key 列表（它是
    vector<string>对象），然后遍历 key 列表逐个访问。
  - 第三， 新增/修改值，新增/修改值时使用操作符=，其参数为 Value 对象，Value
    类构造函数支持上面提到的所有类型，所以操作符=右侧可以直接使用上面提到的类型变量，无需转换。修改某个 JSON
    值时，务必保证新旧的类型一致，否则会抛出逻辑错误的异常。Array 时比较特殊，是调用 append()接口追加，使用下标修改。
  - 第四， 删除，Object 时删除某个 key 使用 removeMember()接口，Array 时删除某个元素使用 removeIndex
    接口指定元素的下标。


# 3. References

- 官网：https://jsoncpp.sourceforge.net/old.html
- 用法参考：https://www.cnblogs.com/ZY-Dream/p/10054074.html