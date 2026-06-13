<!--
 * @Author: JohnJeep
 * @Date: 2021-01-25 09:18:04
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-05-31 19:38:13
 * @Description: xml learning
 * Copyright (c) 2026 by John Jeep, All Rights Reserved. 
-->

# 1. 概念

什么是 XML？
  > XML 一般是指可扩展标记语言，标准通用标记语言的子集，是一种用于标记电子文件使其具有结构性的标记语言。

 开发中使用 XML 主要有以下两方面应用.
  - XML 做为数据交换的载体，用于数据的存储与传输
  - XML 做为配置文件


# 2. 书写规范

- xml 必须有根元素(只有一个)
- xml 标签必须有关闭标签
- xml 标签对大小写敏感
- xml 的属性值须加引号
- 特殊字符必须转义
- xml 中的标签名不能有空格
- 空格/回车/制表符在 xml 中都是文本节点
- xml 必须正确地嵌套


# 3. XML 文件的优点：

1. XML 文档内容和结构完全分离。
2. 互操作性强。
3. 规范统一。
4. 支持多种编码。
5. 可扩展性强。


# 4. 解析 XML 文件及优缺点

一般有 DOM 解析和 SAX 解析。

DOM 解析优缺点
- 优点
  - 允许应用程序对数据和结构做出更改。
  - 访问是双向的，可以在任何时候在树中上下导航，获取和操作任意部分的数据。
- 缺点
  - 通常需要加载整个 XML 文档来构造层次结构，消耗资源大。


# 5. References

- [菜鸟 Xml 教程](https://www.runoob.com/xml/xml-syntax.html)