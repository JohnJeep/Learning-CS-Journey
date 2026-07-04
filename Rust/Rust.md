<!--
 * @Author: JohnJeep
 * @Date: 2026-07-04 16:55:33
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-07-04 17:30:41
 * @Description: rust usage
 * Copyright (c) 2026 by John Jeep, All Rights Reserved. 
-->

# Introduction

rust 是一门系统级编程语言，类似 C/C++，但更安全、更现代化。它的主要特点是：

- **内存安全**：通过所有权系统（ownership system）和借用检查器（borrow checker）在编译期保证内存安全，避免悬空指针、越界访问等问题。
- **零成本抽象**：高层抽象不会带来额外的运行时开销，性能接近 C/C++。
- **并发安全**：通过所有权和类型系统防止数据竞争，轻松编写安全的并发程序。
- **现代化语法**：语法简洁，支持模式匹配、泛型、闭包等现代编程特性。
- **丰富的生态**：Cargo 包管理器和 crates.io 提供了丰富的库和工具，方便快速开发。



# Rustc

Rust 的编译器叫 **rustc**（Rust Compiler）。

- rustc 是 Rust 官方、唯一的主流编译器（不像 C++ 有 GCC、Clang、MSVC 好几家竞争）
- 用 Rust 自身写成（自举，self-hosted），前端基于 LLVM 做后端代码生成——这点和 Clang 类似，都是"自己的前端 + LLVM 后端"
- 平时你不会直接手敲 `rustc` 命令，而是通过 **Cargo** 间接调用它。`cargo build` 底层其实就是拼好参数去调 `rustc`，就像 CMake 底层调 GCC/Clang 一样

**和 C++ 工具链的对应关系**

| C++                       | Rust                   |
| ------------------------- | ---------------------- |
| GCC / Clang（编译器本体） | rustc                  |
| CMake（构建系统）         | Cargo                  |
| Clang 前端 + LLVM 后端    | rustc 前端 + LLVM 后端 |

**安装方式**

Rust 官方推荐用 **rustup** 来管理 rustc 和 Cargo 的版本，类似于 Go 里切换 Go 版本、或者 Python 里用 pyenv 的感觉：

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

装完之后 `rustc` 和 `cargo` 两个命令都会有了，`rustup` 本身还能管理 stable/beta/nightly 三个发布通道，也能装不同的编译目标。

**编译器的一些特点**

rustc 的检查非常严格，尤其是所有权（ownership）和借用检查器（borrow checker）——这也是 Rust 内存安全保证的核心机制，
很多在 C++ 里运行时才炸的问题（悬空指针、越界访问、use-after-free 等），在 Rust 里编译期就会被拦下来。
代价是编译速度通常比 C++ 慢一些，报错信息则以"啰嗦但有用"著称，经常会直接在报错里给出修复建议。

