<!--
 * @Author: JohnJeep
 * @Date: 2025-03-15 13:18:26
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-07-06 00:16:13
 * @Description: TypeScript learning
 * Copyright (c) 2025 by John Jeep, All Rights Reserved. 
-->

# 1. TypeScript Tutorial

TypeScript is a typed superset of JavaScript that compiles to plain JavaScript. It offers classes, modules, and
interfaces to help you build robust components.

TypeScript 是一种有类型检查的 JavaScript。**类型只在编译期存在，运行时是纯 JavaScript**。

这意味着：
- 编译器 `tsc` 做类型检查，然后把 `.ts` 擦除成 `.js`
- 运行时没有 `int`、`string` 这些类型，全部消失
- 你写的类型注解是给开发者和编译器看的，不是给 CPU 看的


# 关键字

TypeScript 的关键字数量没有一个绝对固定的数字，因为它本身分几个层级，以下是完整的分类统计：

TypeScript 包含所有 JavaScript 关键字，并在此基础上增加了自己的关键字。总计大约 **70+ 个**，分为以下四类：

------

## 第一类：JavaScript 保留关键字（45个）

这些在 TS 中完全继承，**不可用作标识符**：

| 控制流                          | 声明                                                         | 值字面量              | 操作符/其他           |
| ------------------------------- | ------------------------------------------------------------ | --------------------- | --------------------- |
| `if` `else`                     | `var` `let` `const`                                          | `true` `false` `null` | `typeof` `instanceof` |
| `for` `while` `do`              | `function` `class`                                           |                       | `in` `of`（循环中）   |
| `switch` `case` `default`       | `import` `export`                                            |                       | `delete` `void`       |
| `break` `continue` `return`     | `extends`                                                    |                       | `new` `this` `super`  |
| `try` `catch` `finally` `throw` | `enum`                                                       |                       | `with` `debugger`     |
| `yield`                         | `package`                                                    |                       | `as`                  |
| `async` `await`                 | `static` `public` `private` `protected` `implements` `interface` |                       |                       |

------

## 第二类：TypeScript 专有关键字（17个）

这些是 TypeScript 在 JavaScript 基础上新增的关键字：`abstract`、`any`、`async`、`await`、`constructor`、`declare`、`from`、`get`、`is`、`module`、`namespace`、`never`、`require`、`set`、`type`、`unknown`、`readonly`。

对 C++ 开发者而言，其中最重要的几个：

| 关键字      | 作用                   | C++ 类比            |
| ----------- | ---------------------- | ------------------- |
| `abstract`  | 抽象类/方法            | `= 0` 纯虚函数      |
| `readonly`  | 只读属性               | `const` 成员        |
| `declare`   | 声明外部类型（无实现） | `.h` 头文件声明     |
| `namespace` | 命名空间               | `namespace`         |
| `is`        | 类型守卫返回值         | 无对应              |
| `never`     | 永不可达类型           | 无对应              |
| `unknown`   | 安全版 any             | `void*`（但更安全） |
| `type`      | 类型别名               | `using` / `typedef` |



## 第三类：上下文关键字（4个）

这些词在特定上下文中有特殊含义，但并非保留字，可以在其他地方用作变量名：`of`（用于 `for...of` 循环）、`asserts`（类型断言）、`infer`（条件类型中推断）、`keyof`（获取类型的键联合）。

```typescript
// keyof 示例：获取对象所有 key 组成的联合类型
type Keys = keyof { name: string; age: number }  // "name" | "age"

// infer 示例：在条件类型中推断
type ReturnType<T> = T extends (...args: any[]) => infer R ? R : never
```

------

## 第四类：内置基础类型名（7个）

虽然严格来说不算关键字，但这些是 TypeScript 的内置类型：`boolean`、`number`、`string`、`symbol`、`object`、`undefined`、`bigint`。它们可以用作变量名（但强烈不建议），这是和 C++ 内置类型的一个区别。

```typescript
// 这在 TS 里合法（但是坏习惯）
let string = "hello"   // ⚠️ 遮蔽了内置类型名
let number = 42        // ⚠️ 同上
```

------

## 数量汇总

| 分类          | 数量    | 能否用作变量名           |
| ------------- | ------- | ------------------------ |
| JS 保留关键字 | ~45     | ❌ 不能                   |
| TS 专有关键字 | 17      | ❌ 不能                   |
| 上下文关键字  | 4       | ⚠️ 可以，但不建议         |
| 内置类型名    | 7       | ⚠️ 技术上可以，强烈不建议 |
| **合计**      | **~73** |                          |

------

作为有 C++/Go 基础的开发者，大多数关键字一看就懂。真正需要重点理解的是 TS 专有的那17个，
尤其是 `type`、`interface`、`declare`、`readonly`、`never`、`unknown`、`is`、`infer`、`keyof` 这9个——它们才是 TypeScript 类型系统的核心武器。



# 2. 工程构建流程

从零开始写一个 TypeScript Hello World，非常简单，下面一步步来：

## 2.1. 初始化项目

```bash
mkdir hello-ts && cd hello-ts
npm init -y
```

## 2.2. 安装 TypeScript

```bash
npm install typescript --save-dev
```

## 2.3. 初始化 TypeScript 配置

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

## 2.4. 写代码

新建 `src/index.ts`：

```typescript
function greet(name: string): string {
  return `Hello, ${name}!`;
}

const message: string = greet("World");
console.log(message);
```

这里用了 TypeScript 最基础的特性：**类型注解**（`: string`）。

## 2.5. 编译并运行

直接用编译器 `tsc` 编译， 一般用的少
```bash
npx tsc          # 编译 → 生成 dist/index.js
node dist/index.js
```

输出：
```
Hello, World!
```

其它的方式：

方式一：用 `tsx`（更快，基于 esbuild）
```bash
# 直接运行的方式
npm install tsx --save-dev
npx tsx src/index.ts
```


或自动编译后，能直接看到运行的结果，一步完成：
```bash
npm install -g tsx

# 运行（自动监听变化）
tsx watch your-file.ts
```

方式二：用 `tsc -w` 让编译器监听文件变化，自动编译：
```bash
npx tsc -w
```


## 2.6. 最终目录结构

```bash
hello-ts/
├── src/
│   └── index.ts       ← 你写的代码
├── dist/
│   └── index.js       ← 编译后的产物
├── tsconfig.json
└── package.json
```

整个流程就这些。TypeScript 的核心价值就是在 `.ts` 文件里加类型，编译成普通 JS 运行——你已经上手了。



# 3. Compile

`tsc` 是 typescript 的编译器。


# 6. 数据类型

Tyscript 中包含的数据类型

1、拥有 JavaScript 中所有的 8 种数据类型

```
string
number
boolean
null
undefined
bigint
symbol
object: 包含 Array, Function, Date, Error
```

2、6 个新类型

- any: 任意类型
  - unknown: 类型安全的 any
- never
    - 什么值都不能有，不是用来限制变量;
    - 用来限制函数的返回值，一般是很特殊的函数，函数是出不来的，比如死循环或函数中抛出异常。
    - `never` 一般是 `typescript` 推断出来的。
  - void
    - 用于函数的返回值, 表示没有返回值;
    - `void` 包含了 `undefined`，但 `undefined` 不一定是 `void`;
    - void 与 undefined 的区别：**`undefined`**：是一个具体的**值类型**，只有一个值 `undefined`。`void`：是一个**语义类型**，专门用来描述"函数没有有意义的返回值"，是关于**意图**的类型，不是关于值的类型。
      ```typescript
      let a: undefined = undefined;  // ✅ 只能赋值 undefined
      let b: void = undefined;       // ✅ 也可以，因为 undefined 是 void 的子类型
      let c: void = 5;               // ❌ 报错，void 不能是别的值
      
      function f1(): undefined {
          return undefined;  // 必须显式返回 undefined
      }
      
      function f2(): void {
          // 什么都不返回也行，返回 undefined 也行
          console.log("hi");
      }
      ```
  - tuple
  - enum

```typescript
// 基本类型
let age: number = 30
let name: string = "Alice"
let active: boolean = true
let nothing: null = null
let unset: undefined = undefined

// any：关掉类型检查（能不用就不用）
let x: any = 5
x = "hello"  // OK

// unknown：比 any 安全，使用前必须检查类型
let y: unknown = fetchData()
if (typeof y === "string") {
  console.log(y.toUpperCase())  // 这里才能用 string 方法
}

// never：永远不会有值的类型（如抛异常的函数返回值）
function fail(msg: string): never {
  throw new Error(msg)
}

// 数组
let nums: number[] = [1, 2, 3]
let strs: Array<string> = ["a", "b"]

// 元组（固定长度、固定类型的数组，类似 C++ std::pair/tuple）
let pair: [string, number] = ["age", 30]

// 对象类型（内联）
let user: { name: string; age: number } = { name: "Bob", age: 25 }
```

**类型推断**（能推断就不写）：

```typescript
let count = 0        // 推断为 number
const msg = "hello"  // 推断为字面量类型 "hello"
```

3、2 个用于自定义类型的方式
  - type
  - interface

```typescript
// interface —— 描述对象结构（推荐用于对象）
interface User {
  id: number
  name: string
  email?: string   // ? 表示可选，类似 C++ std::optional
  readonly createdAt: Date  // readonly 等价于 C++ const 成员
}

// type alias —— 可以描述任何类型
type ID = string | number   // 联合类型
type Point = { x: number; y: number }
type Callback = (err: Error | null, data: string) => void

// 关键区别：
// - interface 可以被 extends 和 implements
// - type 可以表达联合类型、交叉类型等复杂结构
// - 对于对象形状，两者基本可以互换

// 接口继承（类似 C++ 多继承）
interface Animal { name: string }
interface Flyable { fly(): void }
interface Bird extends Animal, Flyable {
  wingspan: number
}

// 类实现接口（类似 C++ 纯虚类）
class Eagle implements Bird {
  name = "eagle"
  wingspan = 220
  fly() { console.log("flying") }
}
```



**联合类型**

这是 TypeScript 最有特色的部分

```typescript
// 联合类型：值可以是多种类型之一
type StringOrNumber = string | number

function format(val: StringOrNumber): string {
  // 类型收窄（Type Narrowing）—— TS 会跟踪每个分支的类型
  if (typeof val === "string") {
    return val.toUpperCase()  // 这里 val 是 string
  }
  return val.toFixed(2)       // 这里 val 是 number
}

// 判别联合类型（Discriminated Union）—— 非常有用！
// 类似 C++ 的 std::variant + std::visit
type Shape =
  | { kind: "circle"; radius: number }
  | { kind: "rect"; width: number; height: number }

function area(s: Shape): number {
  switch (s.kind) {
    case "circle": return Math.PI * s.radius ** 2
    case "rect":   return s.width * s.height
  }
}

// 类型守卫（自定义类型检查函数）
function isString(val: unknown): val is string {
  return typeof val === "string"
}
```


# 变量

var: 有函数作用域；

let/const: 块作用域；



# 泛型

```typescript
// 基础泛型函数（类比 C++ template function）
function identity<T>(arg: T): T {
  return arg
}
identity<string>("hello")  // 显式指定
identity(42)               // 类型推断，T = number

// 泛型约束（类比 C++ concept）
interface HasLength { length: number }
function logLength<T extends HasLength>(arg: T): T {
  console.log(arg.length)
  return arg
}
logLength("hello")   // ✅ string 有 length
logLength([1, 2, 3]) // ✅ array 有 length
// logLength(42)     // ❌ number 没有 length

// 泛型接口
interface Stack<T> {
  push(item: T): void
  pop(): T | undefined
  size(): number
}

// 泛型类
class TypedStack<T> implements Stack<T> {
  private items: T[] = []
  push(item: T) { this.items.push(item) }
  pop() { return this.items.pop() }
  size() { return this.items.length }
}

const numStack = new TypedStack<number>()
numStack.push(1)
```


# 多线程

C++ 中并发靠 `std::thread`，TypeScript 是**单线程 + 事件循环**，并发靠 `async/await` + `Promise`，更像 Go 的 goroutine 但只有一个线程。

```typescript
// Promise（类比 C++ std::future，但用法完全不同）
function fetchUser(id: number): Promise<User> {
  return fetch(`/api/users/${id}`)
    .then(res => res.json())
}

// async/await（推荐写法，让异步代码看起来像同步）
async function getUser(id: number): Promise<User> {
  const res = await fetch(`/api/users/${id}`)
  if (!res.ok) throw new Error("Not found")
  return res.json()
}

// 错误处理
async function main() {
  try {
    const user = await getUser(1)
    console.log(user.name)
  } catch (err) {
    console.error(err)
  }
}

// 并发执行（不是多线程，是并发发起 IO，等待最快完成）
async function loadAll() {
  const [user, posts] = await Promise.all([
    getUser(1),
    getPosts(1)
  ])
  return { user, posts }
}
```


# 7. References

- Microsoft official Document: https://www.typescriptlang.org
- TypeScript 入门教程: https://ts.xcatliu.com/introduction/index.html
