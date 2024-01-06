<!--
 * @Author: JohnJeep
 * @Date: 2023-12-18 14:05:02
 * @LastEditors: JohnJeep
 * @LastEditTime: 2024-01-06 17:44:45
 * @Description: Go 测试用法
 * Copyright (c) 2024 by John Jeep, All Rights Reserved. 
-->

- [1. Testing(测试)](#1-testing测试)
  - [1.1. 测试函数](#11-测试函数)
  - [1.2. 基准测试](#12-基准测试)
  - [1.3. Coverage(测试覆盖率)](#13-coverage测试覆盖率)
- [2. 为测试做设计](#2-为测试做设计)
    - [2.0.1. 对代码阅读的思考](#201-对代码阅读的思考)
- [3. References](#3-references)

# 1. Testing(测试)

Go 语言的工具和标准库中集成了轻量级的测试功能，避免了强大但复杂的测试框架。测试库提供了一些基本构件，必要时可以用来构建复杂的测试构件。

go test 命令是一个按照一定的约定和组织来测试代码的程序。 在包目录内， 所有以 `_test.go` 为后缀名的源文件在执行 `go build` 时不会被构建成包的一部分， 它们是 `go test` 测试的一部分。

go test 命令执行原理

> go test 命令会遍历所有的 *_test.go 文件中符合上述命名规则的函数， 生成一个临时的 main 包用于调用相应的测试函数， 接着构建并运行、 报告测试结果， 最后清理测试中生成的临时文件。


## 1.1. 测试函数

- 测试用例文件名必须以 `_test.go` 结尾，例如：`socket_test.go`

- 在 `*_test.go` 文件中， 分为三种类型的函数： 测试函数、 基准测试 (benchmark) 函数、 示例函数。 一个测试函数是以 `Test` 为函数名前缀的函数， 用于测试程序的一些逻辑行为是否正确，go test 命令会调用这些测试函数并报告测试结果是 `PASS` 或 `FAIL`。 基准测试函数是以 `Benchmark` 为函数名前缀的函数， 它们用于衡量一些函数的性能； go test 命令会多次运行基准函数以计算一个平均的执行时间。

- 每个测试用例中必须导入 `testing` 包。

  ```go
  import "testing"
  ```

- 测试用例中的函数名必须以 `Test` 开头，可选的后缀名必须以大写字母开头 。常见的命名原则：`Test + 被测试的函数名 `。测试用例函数 `TestSocket(t *testing.T)` 的形参必须是 `*testing.T`

  ```go
  func TestSocket(t *testing.T) { /* ..*/}
  func TestSin(t *testing.T) { /* ... */ }
  func TestCos(t *testing.T) { /* ... */ }
  func TestLog(t *testing.T) { /* ... */ }
  ```

  其中 `t` 参数用于报告测试失败和附加的日志信息。

- 一个测试用例文件中可以有多个测试用例函数，比如：`TestAdd()`、`TestSub()` 等。

- 测试用例运行指令：

  ```go
  // 若运行正确，则无日志；若运行错误，则有日志
  // go test 后面没有参数指定包，那么将默认采用当前目录对应的包
  go test
  
  // 无论运行是正确还是错误，都有日志输出
  // 参数 -v: 显示每个测试用例的结果和运行时间
  go test -v
  
  // 测试单个文件
  go test -v socket_test.go socket.go
  
  // 测试单个方法
  go test -v -test.run TestAdd
  
  // CPU 剖析数据标识了最耗 CPU 时间的函数。 在每个 CPU 上运行的线程在每隔几毫秒都会遇到
  // 操作系统的中断事件， 每次中断时都会记录一个剖析数据然后恢复正常的运行。
  go test -cpuprofile=cpu.out
  
  // 堆剖析则标识了最耗内存的语句。剖析库会记录调用内部内存分配的操作，
  // 平均每 512KB 的内存申请会触发一个剖析数据。
  go test -memprofile=mem.out
  
  // 阻塞剖析则记录阻塞 goroutine 最久的操作， 例如系统调用、 管道发送和接收，
  // 还有获取锁等。 每当 goroutine 被这些操作阻塞时， 剖析库都会记录相应的事件。
  go test -blockprofile=block.out
  ```

  若同时开启 CPU 剖析、堆剖析、阻塞剖析时，要当心， 因为某一项分析操作可能会影响其他项的分析结果 。

- 当出现错误时，用 `t.Fatalf` 来格式化输出错误信息，并退出程序。

- `t.Logf` 可在测试用例中输出日志。

- 测试用例函数，并没有放在 `main` 函数中，但用例程序也执行。因为在导入测试用例的包时，Golang 的测试框架中有 `main` 开始执行，然后再加载 测试用例程序 `xxx_test.go` 和 业务程序 `xxx.go`。

- 测试用例程序执行完全后，显示的结果是 `PASS` 或者 `FAIL`。其中：`PASS` 表示测试用例运行成功；`FAIL` 表示测试用例执行失败。测试用例文件中的函数都执行成功，才显示 `PASS`；若发现有一个执行不成功，也会显示 `FAIL`。


## 1.2. 基准测试

基准测试是测量一个程序在固定工作负载下的性能。 在 Go 语言中， 基准测试函数和普通测试函数写法类似， 但是以 Benchmark 为前缀名， 并且带有一个 `*testing.B` 类型的参数； `*testing.B` 参数除了提供和 `*testing.T` 类似的方法， 还有额外一些和性能测量相关的方法。 **它还提供了一个整数 `N`， 用于指定操作执行的循环次数。**

```go
import "testing"

// 循环中执行 N 次
func BenchmarkSum(b *testing.B) {
	for i := 0; i < b.N; i++ {
		Sum("A man, a plan, a canal: Panama")
	}
}
```

运行基准测试：

基准测试与普通的测试不同，默认情况下不运行任何基准测试。要运行基准测试，使用 `-bench` 命令参数，该参数是一个正则表达式， 用于匹配要执行的基准测试函数的名字， 默认值是空的。 其中 `.` 模式将可以匹配所有
基准测试函数 。

```go
// 匹配当前路径下所有的基准测试函数
go test -bench=.

goos: linux
goarch: amd64
pkg: test/product
cpu: Intel(R) Core(TM) i5-10400H CPU @ 2.60GHz
BenchmarkBuy-8
...
1000000000               0.0001251 ns/op
PASS
ok      test/product    0.007s

// 只运行某个基准函数
got test -v -bench="基准函数名"
```

基准测试结果中 `BenchmarkBuy-8`，基准测试函数的后缀部分 `8`，表示运行时对应的 `GOMAXPROCS` 值， 这对
于一些与并发相关的基准测试是重要的信息。

`-benchmem` 命令行参数：提供每次操作分配内存的次数，以及总共分配内存的字节数。

```go
go test -bench=. -benchmem

goos: linux
goarch: amd64
pkg: test/word
cpu: Intel(R) Core(TM) i5-10400H CPU @ 2.60GHz
BenchmarkIsPalindrome02-8        4092888               281.4 ns/op           248 B/op          5 allocs/op
BenchmarkIsPalindrome03-8        4306605               277.4 ns/op           248 B/op          5 allocs/op
BenchmarkIsPalindrome04-8        7649563               147.7 ns/op           128 B/op          1 allocs/op
PASS
ok      test/word       4.225s
```

优化后的 `BenchmarkIsPalindrome04` 函数从从之前的 5 次内存分配变成了一次内存分配 `1 allocs/op`。

注：

- 单位为 `allocs/op` 的值表示每次操作从堆上分配内存的次数。
- 单位为 `B/op` 的值表示每次操作分配的字节数。


比较型的基准测试就是普通程序代码。 它们通常是单参数的函数， 由几个不同数量级的基准测试函数调用， 就像这样：

```go
func benchmark(b *testing.B, size int) { /* ... */ }
func Benchmark10(b *testing.B) { benchmark(b, 10) }
func Benchmark100(b *testing.B) { benchmark(b, 100) }
func Benchmark1000(b *testing.B) { benchmark(b, 1000) }
```

通过函数参数来指定输入的大小， 但是参数变量对于每个具体的基准测试都是固定的。 要避免直接修改 `b.N` 来控制输入的大小。 除非你将它作为一个固定大小的迭代计算输入， 否则基准测试的结果将毫无意义。


## 1.3. Coverage(测试覆盖率)

执行 go 的测试程序时，用 `coverage` 来统计测试用例的覆盖率。

```go
go test -v -run=Coverage example/word
```

 执行 `example/word` 路径下所有 go 测试用例，并统计测试用例的覆盖率。

查看 测试用例覆盖攻击的使用方法，可用 `go tool cover` 命令。

```go
Usage of 'go tool cover':
Given a coverage profile produced by 'go test':
        go test -coverprofile=c.out

Open a web browser displaying annotated source code:
        go tool cover -html=c.out

Write out an HTML file instead of launching a web browser:
        go tool cover -html=c.out -o coverage.html

Display coverage percentages to stdout for each function:
        go tool cover -func=c.out

Finally, to generate modified source code with coverage annotations
(what go test -cover does):
        go tool cover -mode=set -var=CoverageVariableName program.go

Flags:
  -V    print version and exit
  -func string
        output coverage profile information for each function
  -html string
        generate HTML representation of coverage profile
  -mode string
        coverage mode: set, count, atomic
  -o string
        file for output; default: stdout
  -var string
        name of coverage variable to generate (default "GoCover")

  Only one of -html, -func, or -mode may be set.
```

go tool 命令运行 Go 工具链的底层可执行程序。 这些底层可执行程序放在 ` $GOROOT/pkg/tool/${GOOS}_${GOARCH}` 目录。 因为有 go build 命令的原因， 我们很少直接调用这些底层工具。

现在我们可以用 `-coverprofile` 标志参数重新运行测试：

```go
go test -run=Coverage -coverprofile=c.out  example/word
```

 `-coverprofile` 在测试代码中插入生成钩子来统计覆盖率数据。 也就是说， 在运行每个测试前，它将待测代码拷贝一份并做修改， 在每个词法块都会设置一个布尔标志变量. 当被修改后的被测试代码运行退出时，将统计日志数据写入 `c.out` 文件， 并打印一部分执行的语句的一个总结。（ 如果你需要的是摘要， 使用 go test -cover 。）

为了收集数据， 我们运行了测试覆盖率工具， 打印了测试日志， 使用下面的命令，生成一个 HTML 报告， 然后在浏览器中打开  。

```go
go tool cover -html=c.out
```



# 2. 为测试做设计

1. 测试编码规范
2. 测试编码流程
3. 测试编码工具
4. 测试平台的思考和构建

坚持的原则：尽早测试，经常测试，自动测试。一旦代码写出来，就要尽早开始测试。



### 2.0.1. 对代码阅读的思考

- 在代码里把事情讲明白，让人能快速理解他人的代码，就能快速做出修改的决策。“猜测他人代码的逻辑用意”是很难受且困难的，他人的代码也会在这种场景下，产生被误读。
- 一般阅读他人的代码时，可能有些没有必要的地方，别人要花很多的脑子猜“为什么是这样的？”
- 代码写的原则是：简单、易理解、逻辑清晰。
- 写代码前先理清基本的需求、设计以及实现的流程。
- **不要面向需求编程，要面向业务模型编程。把变更的需求看成是业务模型的一个可变的参数，类似于函数的参数一样。**
- **注释要把问题讲清楚, 讲不清楚的日志等于没有**



# 3. References

- [腾讯工作13年之所思所想，那些优秀程序员的共性特征](https://mp.weixin.qq.com/s/FKRedldguFVPred7johg8A)
- [腾讯 13 年，我所总结的Code Review终极大法](https://mp.weixin.qq.com/s/HoFSNCd1U3eoUqYaQiEgwQ)

