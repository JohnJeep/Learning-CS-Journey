---
title: 25-atoi()
data: 2025-03-30 00:04:11
tags: ['C']
category: C
---

- atoi (表示 ascii to integer)是把字符串转换成整型数的一个函数
- 函数原型：`int atoi(const char *nptr);`
  - 函数会扫描参数 nptr字符串，会跳过前面的空白字符（例如空格，tab缩进）等
  - 如果 nptr不能转换成 int 或者 nptr为空字符串，那么将返回 0



