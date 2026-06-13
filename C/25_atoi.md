<!--
 * @Author: JohnJeep
 * @Date: 2020-01-17 11:20:34
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-05-31 19:30:10
 * @Description: atoi in C language
 * Copyright (c) 2026 by John Jeep, All Rights Reserved. 
-->

- atoi (表示 ascii to integer)是把字符串转换成整型数的一个函数
- 函数原型：`int atoi(const char *nptr);`
  - 函数会扫描参数 nptr 字符串，会跳过前面的空白字符（例如空格，tab 缩进）等
  - 如果 nptr 不能转换成 int 或者 nptr 为空字符串，那么将返回 0
