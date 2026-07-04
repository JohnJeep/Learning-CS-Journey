<!--
 * @Author: JohnJeep
 * @Date: 2026-07-04 16:52:51
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-07-04 17:28:58
 * @Description: rust cargo usage
 * Copyright (c) 2026 by John Jeep, All Rights Reserved. 
-->

**Cargo** 是 Rust 的官方构建工具和包管理器，类似于：

- C++ 里的 CMake + vcpkg/Conan（构建 + 依赖管理二合一）
- Golang 里的 `go build` + `go mod`

**它主要做这几件事**

1. **依赖管理**：项目根目录下的 `Cargo.toml` 文件里声明依赖的库（叫 "crate"），Cargo 会自动从 crates.io（Rust 的包仓库，类似 npm 的 registry）下载并解析版本冲突。
2. **构建项目**：`cargo build` 编译项目，`cargo build --release` 编译优化版本（类似 CMake 的 Release 模式）。
3. **运行/测试**：`cargo run` 编译并运行，`cargo test` 跑测试，一条命令搞定，不用自己拼编译命令。
4. **格式化和检查**：`cargo fmt` 格式化代码（类似 clang-format），`cargo clippy` 做静态检查（类似 clang-tidy，但更严格，会给出 Rust 惯用写法的建议）。
5. **文档生成**：`cargo doc` 直接从代码注释生成文档网站，这也是 `docs.rs` 上那些库文档的来源。

**和 C++/Go 生态的对比**

| 功能     | C++              | Go            | Rust (Cargo)       |
| -------- | ---------------- | ------------- | ------------------ |
| 构建     | CMake/Make       | `go build`    | `cargo build`      |
| 依赖管理 | vcpkg/Conan/手动 | `go mod`      | 内置，`Cargo.toml` |
| 包仓库   | 分散（各种源）   | 直接从 VCS 拉 | 统一的 crates.io   |
| 版本锁定 | 视工具而定       | `go.sum`      | `Cargo.lock`       |

Cargo 几乎是"开箱即用"的一体化工具，不像 C++ 那样需要自己拼凑 CMake + 包管理器 + 各种插件。日常开发时，基本上 `cargo build`、`cargo run` 这两个命令就能覆盖大部分日常需求。

