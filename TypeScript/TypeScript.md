<!--
 * @Author: JohnJeep
 * @Date: 2025-03-15 13:18:26
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-03-25 00:27:33
 * @Description: TypeScript learning
 * Copyright (c) 2025 by John Jeep, All Rights Reserved. 
-->
# TypeScript Tutorial

TypeScript is a typed superset of JavaScript that compiles to plain JavaScript. It offers classes, modules, and interfaces to help you build robust components.


## Compile
安装 typescript 编译器，终端执行下面的命令，前提是已经安装好 `npm` 软件包。
```shell
sudo npm i typescript -g 
```

静态编译，每次文件改变后，终端运行 TypeScript 编译器进行编译
```shell
tsc [filename]
```
不推荐这种方式，这种方式太麻烦了，推荐使用自动化编译。


动态编译 TS 工程文件同时监测内容的变化，自动生成 js 文件
1. 终端输入：`tsc --init`，执行完成后，自动生成一个 `tsconfig.json` 配置文件。此配置文件是对 ts 文件的全局配置。
2. 终端输入：`tsc --watch`，监控所有 js 文件的变化，同时自动将 ts 转换为 js。

## 类型声明


## 类型推断

## 数据类型

JavaScript 中包含 8 种数据类型
1. string
2. number
3. boolean
4. null
5. undefined
6. bigint
7. symbol
8. object: 包含 Array, Function, Date, Error


Tyscript 中包含的数据类型
1. 拥有 JavaScript 中所有的数据类型
2. 6 个新类型
   - any
   - unknown: 类型安全的 any
   - never
      - 什么值都不能有，不是用来限制变量的
      - 要是是用来限制函数的返回值，一般是很特殊的函数，函数是出不来的，比如死循环或函数中抛出异常。
      - never 一般是 typescript 推断出来的。
   - void
      - 用于函数的返回值
      - `void` 包含了 `undefined`，但 `undefined` 不一定是 `void`
   - tuple
   - ernum
3. 2 个用于自定义类型的方式
   - type
   - interface


# References

- Microsoft official Document: https://www.typescriptlang.org
- TypeScript 入门教程: https://ts.xcatliu.com/introduction/index.html