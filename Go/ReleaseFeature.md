# Release Feature

## Go 1.20

## Go 1.19

## Go 1.18

2022年3月15日，Go1.18 版本发布。

- 增加了一个主要的新语言特性: 对泛型的支持。

## Go 1.17



## Go 1.16

2021年2月18日，Go1.16版本发布。

- 支持苹果的 M1 芯片。
- `go build/run` 命令不再自动更新 `go.mod` 和 `go.sum` 文件。
- 新增 `io/fs` 包，建立 Go 原生系统的抽象。
- 新增 embed 包，作为在二进制文件中嵌入静态资源文件的官方方案。
- `GODEBUD`环境变量支持跟踪包 init 函数的消耗。

## Go 1.15

2020年8月12日，Go1.15版本发布。

- module 的本地缓存可通过 `GOMODCACHE` 环境变量配置。
- 增加 tzdata 包，用于操作附加到二进制文件中的时区信息。

## Go 1.14

2020年2月16日，Go 1.14版本发布。

- 重新实现了运行时的 timer。
- testing 包的 T 和 B 类型都增加了自己的 Cleanup 方法。
- 基于信号系统机制实现了异步抢占式的 goroutine 调度。

## Go 1.13

2019年9月4日，Go 1.13版本发布。

- 标准库中新增 `errors.Is` 和 `errors.As` 函数来解决错误值的比较判定，增加 `errors.Unwrap` 函数来解决 error 的展开问题。

## Go 1.12

2019年2月15日，Go1.12版本发布。

- 增加对 TLS 1.3 的支持。
- 对Go module 机制做了进一步优化。
- `Build cache` 默认开始成为必需的功能。

## Go 1.11

2018年8月25日，Go1.11版本发布。这也是一个具有里程碑意义的版本。

- 引入了 Go module 包管理机制。
- 引入了对 web Assemble 的支持。
- 为调试器新增了一个试验的功能---允许在调试过程中动态调用 Go 函数。

## Go 1.10

2018年2月17日，Go 1.10 版本发布。

- 支持默认的 `GOROOT`，开发者无需显示的设置 `GOROOT` 的环境变量。
- 通过 cache 大幅度提升构建和 go test 的执行性能，并基于源文件中内容的变化与否，判断是否使用cache中的结果。
- 增加 `GOTMPDIR` 环境变量。

## Go 1.9

2017年8月25日，Go 1.9 颁布发布。

- 新增 type alias 语法。
- 增加支持并发的 Map 类型---`sync.map`。
- 增加 `math/bits` 包。

## Go 1.8

2017年2月16日，Go 1.8 版本发布。

- 标准库中增加 `sort.Slice` 函数。
- 支持 HTTP/2 push 机制
- 支持HTTP Server 优雅退出。
- 增加了对 Mutex 和 RWMutex 的profiling 的支持。

## Go 1.7

2016年8月15日，Go1.7版本发布。

- go test 支持 subtests 和 sub-benchmarks。
- 标准库新增 context 包。

## Go 1.6

2016年2月17日，Go 1.6版本发布。

- 支持 HTPTP/2
- 定义了在C代码中共享 Go 指针的规则。

## Go 1.5

2015年8月19日，Go 1.5 版本发布。是Go 语言历史上具有里程碑意义的重要版本。

- Go 实现了自举。Go 语言的编译器和运行时全部使用Go重写，原先C代码实现被彻底移除。
- 增加 go tool trace 子命令。
- go build 增加 `-buildmode` 命令选项，支持将go 代码编译为共享库（shared library）的形式。
- 支持 map 类型字面量（literal）。



# References

- Go 官方Release History：https://go.dev/doc/devel/release

