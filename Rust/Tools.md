<!--
 * @Author: JohnJeep
 * @Date: 2026-07-04 16:52:51
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-11-14 11:02:54
 * @Description: rust cargo usage
 * Copyright (c) 2026 by John Jeep, All Rights Reserved.
-->

# 1. rustup

rustup 是 rust 的工具链（toolchain）的安装器。

管理多个版本
```bash
rustup install stable / nightly / 1.75.0   # 装不同版本
rustup default stable                       # 设默认
rustup override set nightly                 # 给某个项目单独指定版本
rustup update                               # 更新工具链
```

管理组件（component）
```bash
rustup component add rustfmt clippy   # 装格式化/lint 工具
rustup component add rust-src         # 装标准库源码
```

管理目标平台（target）：交叉编译时用
```bash
rustup target add x86_64-pc-windows-gnu
```

关键点：rustup 本身不编译代码，它只负责"把 rustc、cargo、rustfmt 等工具装好、切换好版本"。装完 Rust 后你平时几乎不用管它，除非要切版本/加组件。


# 2. Cargo

**Cargo** 是 Rust 的官方构建工具和包管理器，类似于：C++ 里的 CMake + vcpkg/Conan（构建 + 依赖管理二合一）,Golang 里的 `go build` + `go mod`。

它主要做这几件事
1. **依赖管理**：项目根目录下的 `Cargo.toml` 文件里声明依赖的库（叫 "crate"），Cargo 会自动从 crates.io（Rust
   的包仓库，类似 npm 的 registry）下载并解析版本冲突。
2. **构建项目**：`cargo build` 编译项目，`cargo build --release` 编译优化版本（类似 CMake 的 Release 模式）。
3. **运行/测试**：`cargo run` 编译并运行，`cargo test` 跑测试，一条命令搞定，不用自己拼编译命令。
4. **格式化和检查**：`cargo fmt` 格式化代码（类似 clang-format），`cargo clippy` 做静态检查（类似
   clang-tidy，但更严格，会给出 Rust 惯用写法的建议）。
5. **文档生成**：`cargo doc` 直接从代码注释生成文档网站，这也是 `docs.rs` 上那些库文档的来源。

**和 C++/Go 生态的对比**

| 功能     | C++              | Go            | Rust (Cargo)       |
| -------- | ---------------- | ------------- | ------------------ |
| 构建     | CMake/Make       | `go build`    | `cargo build`      |
| 依赖管理 | vcpkg/Conan/手动 | `go mod`      | 内置，`Cargo.toml` |
| 包仓库   | 分散（各种源）   | 直接从 VCS 拉 | 统一的 crates.io   |
| 版本锁定 | 视工具而定       | `go.sum`      | `Cargo.lock`       |

Cargo 几乎是"开箱即用"的一体化工具，不像 C++ 那样需要自己拼凑 CMake + 包管理器 + 各种插件。日常开发时，基本上 `cargo
build`、`cargo run` 这两个命令就能覆盖大部分日常需求。


核心文件：
- `Cargo.toml`：项目配置 + 依赖声明（类似 package.json）。cargo 特有的项目数据描述文件。它存储了项目的所有元配置信息，如果 Rust 开发者希望 Rust 项目能够按照期望的方式进行构建、测试和运行，那么，必须按照合理的方式构建 Cargo.toml。
- `Cargo.lock`：锁定精确版本（类似 package-lock.json）。是 cargo 工具根据同一项目的 toml 文件生成的项目依赖详细清单，因此我们一般不用修改它，只需要对着 Cargo.toml 文件撸就行了。

关键点：`cargo build` 本质是 "读 Cargo.toml → 解析依赖 → 为每个 crate 调用 rustc → 链接"。你几乎不需要直接碰 rustc。

cargo 背后自动调用了 rustc。


常用 cargo 命令

```bash
# 创建一个新的 package
cargo new <pkg_name>

# 创建的 package 目录结构如下
$ tree
.
├── .git
├── .gitignore
├── Cargo.toml
└── src
    └── main.rs

# 创建一个新的 package，不带 git 版本控制
cargo new <pkg_name> --vcs none

# 构建包
cargo build

# 以 release 模式构建
cargo build --release

#运行包
cargo run

# 以 release 模式运行
cargo run --release

# 快速检查代码是否能编译通过
cargo check

# 引用其它 package 为依赖
cargo add ferris-says

# 格式化代码
cargo fmt

# 工程构建文档
cargo doc

```


# 3. rustc

rustc 是 Rust 的编译器。负责把把 `.rs` 源码编译成机器码的编译器，类似 gcc / javac。

为什么很少直接使用 rustc？
> 直接用它编译多文件/带依赖的项目会非常麻烦（要手动传一堆参数），不方便；
