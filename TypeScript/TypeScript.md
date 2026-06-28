<!--
 * @Author: JohnJeep
 * @Date: 2025-03-15 13:18:26
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-05-31 18:39:16
 * @Description: TypeScript learning
 * Copyright (c) 2025 by John Jeep, All Rights Reserved. 
-->

# TypeScript Tutorial

TypeScript is a typed superset of JavaScript that compiles to plain JavaScript. It offers classes, modules, and
interfaces to help you build robust components.

TypeScript 是一种有类型检查的 JavaScript。**类型只在编译期存在，运行时是纯 JavaScript**。

这意味着：

- 编译器 `tsc` 做类型检查，然后把 `.ts` 擦除成 `.js`
- 运行时没有 `int`、`string` 这些类型，全部消失
- 你写的类型注解是给开发者和编译器看的，不是给 CPU 看的

## 工程构建流程

从零开始写一个 TypeScript Hello World，非常简单，下面一步步来：

### 1. 初始化项目

```bash
mkdir hello-ts && cd hello-ts
npm init -y
```

### 2. 安装 TypeScript

```bash
npm install typescript --save-dev
```

### 3. 初始化 TypeScript 配置

```bash
npx tsc --init
```

这会生成一个 `tsconfig.json`，核心配置如下（默认即可）：

```json
{
  "compilerOptions": {
    "target": "ES2016",
    "module": "commonjs",
    "strict": true,
    "outDir": "./dist",
    "rootDir": "./src"
  }
}
```

> 建议手动加上 `"outDir": "./dist"` 和 `"rootDir": "./src"`，让输出更整洁。

### 4. 写代码

新建 `src/index.ts`：

```typescript
function greet(name: string): string {
  return `Hello, ${name}!`;
}

const message: string = greet("World");
console.log(message);
```

这里用了 TypeScript 最基础的特性：**类型注解**（`: string`）。

### 5. 编译并运行

```bash
npx tsc          # 编译 → 生成 dist/index.js
node dist/index.js
```

输出：

```
Hello, World!
```

------

### 更快的开发方式（可选）

每次改完都要编译再运行比较繁琐，可以用 `ts-node` 直接运行 `.ts` 文件：

```bash
npm install ts-node --save-dev
npx ts-node src/index.ts
```

或者用 `tsx`（更快，基于 esbuild）：

```bash
npm install tsx --save-dev
npx tsx src/index.ts
```

------

### 最终目录结构

```
hello-ts/
├── src/
│   └── index.ts       ← 你写的代码
├── dist/
│   └── index.js       ← 编译后的产物
├── tsconfig.json
└── package.json
```

整个流程就这些。TypeScript 的核心价值就是在 `.ts` 文件里加类型，编译成普通 JS 运行——你已经上手了。



## Compile

`tsc` 是 typescript 的编译器。



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