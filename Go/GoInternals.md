<!--
 * @Author: JohnJeep
 * @Date: 2023-07-10 09:54:50
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-04-15 17:53:44
 * @Description: golang internal fundamental
 * Copyright (c) 2025 by John Jeep, All Rights Reserved. 
-->

# Introduction

golang 官方源码研读。


# 目录结构

```
go
├── .gitattributes
│   └── 用于定义 Git 仓库中文件的属性，例如文件的换行符格式、是否进行二进制文件处理等。
├── .gitignore
│   └── 用于指定 Git 仓库中需要忽略的文件和目录，避免将不必要的文件纳入版本控制。
├── CONTRIBUTING.md
│   └── 包含项目的贡献指南，指导开发者如何向项目贡献代码、报告问题等。
├── LICENSE
│   └── 项目的开源许可证文件，规定了代码的使用、分发和修改规则。
├── PATENTS
│   └── 与项目相关的专利信息文件。
├── README.md
│   └── 项目的说明文档，提供项目的概述、下载安装方法、贡献方式等信息。
├── SECURITY.md
│   └── 项目的安全相关说明，包括如何报告安全漏洞等。
├── codereview.cfg
│   └── 代码审查的配置文件，可能包含代码审查的规则和流程。
├── go.env
│   └── 环境变量配置文件，用于设置 Go 项目的运行环境。
├── lib
│   ├── fips140
│   │   └── 可能包含与 FIPS 140 标准相关的库代码，FIPS 140 是美国联邦信息处理标准。
│   ├── time
│   │   └── 可能包含与时间处理相关的库代码。
│   └── wasm
│       └── 包含与 WebAssembly（Wasm）相关的库代码，用于支持 Go 代码在 WebAssembly 环境中运行。
├── misc
│   ├── cgo
│   │   └── 包含与 cgo 相关的示例代码和工具，cgo 用于在 Go 代码中调用 C 代码。
│   ├── chrome
│   │   └── 可能包含与 Chrome 浏览器相关的工具或示例代码。
│   ├── editors
│   │   └── 包含用于各种编辑器的配置文件或插件，帮助开发者在不同编辑器中更好地开发 Go 代码。
│   ├── go.mod
│   │   └── Go 模块的依赖管理文件，记录项目的依赖信息。
│   ├── go_android_exec
│   │   └── 包含用于在 Android 系统上执行 Go 代码的工具或示例。
│   ├── ios
│   │   └── 包含与 iOS 系统相关的工具或示例代码，用于在 iOS 平台上开发 Go 应用。
│   └── wasm
│       └── 包含与 WebAssembly 相关的工具或示例代码，用于在 Web 环境中运行 Go 代码。
├── .github
│   ├── CODE_OF_CONDUCT.md
│   │   └── 项目的行为准则文件，规定了社区成员在项目中的行为规范。
│   ├── ISSUE_TEMPLATE
│   │   └── 包含问题报告的模板文件，帮助开发者更规范地提交问题。
│   ├── PULL_REQUEST_TEMPLATE
│   │   └── 包含拉取请求的模板文件，指导开发者如何提交高质量的代码变更请求。
│   └── SUPPORT.md
│       └── 项目的支持相关说明，提供获取帮助的途径。
├── api
│   └── 可能包含项目的 API 文档或与 API 相关的代码。
├── doc
│   ├── README.md
│   │   └── 文档目录的说明文档。
│   ├── asm.html
│   │   └── 汇编语言相关的文档。
│   ├── go_mem.html
│   │   └── Go 语言内存管理相关的文档。
│   ├── go_spec.html
│   │   └── Go 语言规范的文档。
│   ├── godebug.md
│   │   └── 与 Go 调试相关的文档。
│   ├── initial
│   │   └── 可能包含项目初期的文档或示例。
│   └── next
│       └── 可能包含项目未来计划或即将发布的功能的文档。
├── src
│   ├── Make.dist
│   │   └── 用于构建项目的 Makefile 脚本，可能包含项目的编译、打包等任务。
│   ├── README.vendor
│   │   └── 关于项目依赖管理中 vendor 目录的说明文档。
│   ├── all.bash
│   │   └── 用于执行一系列构建和测试任务的 Bash 脚本。
│   ├── all.bat
│   │   └── 用于执行一系列构建和测试任务的 Windows Batch 脚本。
│   ├── all.rc
│   │   └── 可能是与资源文件相关的配置文件。
│   ├── archive
│   │   └── 包含与归档文件处理相关的代码，如 zip、tar 等格式的处理。
│   ├── arena
│   │   └── 可能包含与内存竞技场（arena）相关的代码，用于高效的内存管理。
│   ├── bootstrap.bash
│   │   └── 用于项目的初始化和引导的 Bash 脚本。
│   ├── bufio
│   │   └── 包含与缓冲输入输出相关的代码。
│   ├── buildall.bash
│   │   └── 用于构建整个项目的 Bash 脚本。
│   ├── builtin
│   │   └── 包含 Go 语言内置函数和类型的实现代码。
│   └── ...
│       └── 其他可能的子目录，包含不同功能模块的代码。
└── test
    └── 包含项目的测试代码，用于对项目的各个功能进行单元测试、集成测试等。
```


src 目录

```
go/src
├── cmd
│   ├── compile
│   │   ├── internal
│   │   │   ├── noder
│   │   │   │   └── unified.go
│   │   │   │       └── 用于生成包存根文件，包含检查文件、收集声明、写入元数据等操作。
│   │   │   ├── ssa
│   │   │   │   ├── decompose.go
│   │   │   │   │   └── 用于分解用户定义的数组和结构体，处理命名值和嵌套结构。
│   │   │   │   ├── compile.go
│   │   │   │   │   └── 定义约束类型，可能用于 SSA 编译过程中的依赖关系。
│   │   │   │   └── rewrite.go
│   │   │   │       └── 包含规范比较函数，用于规范代码生成过程中的值比较。
│   │   │   ├── types2
│   │   │   │   ├── sizes.go
│   │   │   │   │   └── 实现偏移量计算和对齐函数，处理类型的内存布局。
│   │   │   │   ├── scope.go
│   │   │   │   │   └── 实现查找忽略大小写的对象功能，处理作用域内的对象查找。
│   │   │   │   ├── index.go
│   │   │   │   │   └── 验证索引的有效性，检查索引类型和常量范围。
│   │   │   │   └── predicates.go
│   │   │   │       └── 判断类型是否有空类型集，用于类型检查。
│   │   │   └── inline
│   │   │       └── inlheur
│   │   │           └── analyze.go
│   │   │               └── 分析函数的内联性，处理函数和闭包的内联分析。
│   │   └── test
│   │       └── ssa_test.go
│   │           └── 运行生成测试，包含生成和执行测试代码的逻辑。
│   ├── internal
│   │   ├── obj
│   │   │   ├── plist.go
│   │   │   │   └── 处理程序的入口活动信息，发射入口栈映射和不安全点信息。
│   │   │   └── riscv
│   │   │       └── obj.go
│   │   │           └── 检查立即数是否为偶数，用于 RISC - V 架构的对象处理。
│   │   └── script
│   │       └── state.go
│   │           └── 从存档中提取文件到工作目录，处理文件路径和权限。
│   └── doc
│       └── 实现 `go doc` 工具，用于查看 Go 包文档。
├── go
│   ├── types
│   │   ├── sizes.go
│   │   │   └── 实现偏移量计算和对齐函数，与 `cmd/compile/internal/types2/sizes.go` 类似。
│   │   ├── scope.go
│   │   │   └── 实现查找忽略大小写的对象功能，与 `cmd/compile/internal/types2/scope.go` 类似。
│   │   └── predicates.go
│   │       └── 判断类型是否有空类型集，与 `cmd/compile/internal/types2/predicates.go` 类似。
│   └── doc
│       └── exports.go
│           └── 过滤标识符列表，只保留导出的标识符。
├── math
│   ├── big
│   │   ├── bits_test.go
│   │   │   └── 包含位操作的测试代码，如规范位列表。
│   │   └── float.go
│   │       └── 实现浮点数的设置操作，处理浮点数的精度和值复制。
│   └── rand
│       │   ├── rand.go
│       │   │   └── 创建随机数生成器，基于源生成随机数。
│       │   └── v2
│       │       └── rand.go
│       │           └── 创建随机数生成器，v2 版本的实现。
├── net
│   ├── lookup.go
│   │   └── 实现 DNS 查找功能，如查找 MX、NS、SRV 记录，过滤无效记录。
│   ├── tcpsock.go
│   │   └── 检查 TCP 连接是否使用多路径 TCP。
│   └── dnsclient_unix_test.go
│       └── 测试 DNS 客户端在 Unix 系统上的错误处理，模拟 DNS 服务器响应。
├── os
│   └── exec
│       └── exec.go
│           └── 去重环境变量，处理环境变量的重复问题。
├── flag
│   └── flag.go
│       └── 获取标志集的输出写入器，处理标志集的输出。
├── compress
│   └── bzip2
│       └── bzip2.go
│           └── 实现逆 Burrows - Wheeler 变换，用于数据压缩。
├── runtime
│   ├── preempt.go
│   │   └── 判断是否为异步安全点，处理 goroutine 的异步抢占。
│   ├── traceback_test.go
│   │   └── 解析回溯信息，用于调试和分析 goroutine 调用栈。
│   └── trace.go
│       └── 注册跟踪标签和原因，处理跟踪信息的生成。
├── crypto
│   ├── tls
│   │   ├── handshake_server_test.go
│   │   │   └── 测试 TLS 握手的上下文层次结构，处理客户端和服务器的握手。
│   │   ├── handshake_messages.go
│   │   │   └── 添加带长度的字节序列，处理 TLS 握手消息的编码。
│   │   └── cipher_suites.go
│   │       └── 获取支持的 TLS 密码套件列表，处理 TLS 密码套件的选择。
│   └── internal
│       └── fips140
│           └── aes
│               └── aes_test.go
│                   └── 测试 AES 算法的相关数据，如 S 盒、乘法表等。
├── encoding
│   └── asn1
│       └── asn1.go
│           └── 解析 ASN.1 整数，处理整数的编码和解码。
├── fmt
│   └── scan_test.go
│       └── 测试扫描函数的 EOF 处理，处理输入结束时的错误情况。
├── internal
│   └── coverage
│       └── cfile
│           ├── ts_test.go
│           │   └── 测试覆盖率快照，运行覆盖率测试。
│           └── testsupport.go
│               └── 处理覆盖率测试目录，生成和处理覆盖率数据。
├── test
│   ├── typeparam
│   │   └── graph.go
│   │       └── 实现图的最短路径算法，处理图的路径查找。
│   └── fixedbugs
│       └── issue12588.go
│           └── 包含修复特定 bug 的测试代码。
```

## 程序是怎样跑起来的




# HTTP



# References

- Github golang-internals-resources: https://github.com/emluque/golang-internals-resources
- Github learning go: https://github.com/yangwenmai/learning-golang
- Go 源码阅读工具： https://mp.weixin.qq.com/s/E2TL_kcbVcRJ0CnxwbXWLw
- Github golang-notes, 源码剖析：https://github.com/cch123/golang-notes/tree/master
- Go Context 并发编程简明教程：https://geektutu.com/post/quick-go-context.html
- 曹大的 《Go 程序的启动流程》和全成的 《Go 程序是怎样跑起来的》


---
HTTP

Golang 使用系列---- Net/Http 应用层: https://kingjcy.github.io/post/golang/go-net-http

---
环形双向链表

- Go 标准库-双向链表 (container/list) 源码解析: https://blog.csdn.net/eight_eyes/article/details/121068799
- 链表: 深入理解container/list&LRU缓存的实现: https://lailin.xyz/post/list.html
- 极客书房 Go 数据结构和算法篇（一）链表: https://study.geekai.co/posts/go-data-structure-linked-list 


