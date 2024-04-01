<!--

 * @Author: johnjeep
 * @Date: 2022-12-27 20:41:57
 * @LastEditors: JohnJeep
 * @LastEditTime: 2023-07-21 17:27:43
 * @Description: Go 标准库学习
 * Copyright (c) 2022 by johnjeep, All Rights Reserved.
-->
<!-- TOC -->

- [1. GO Standard library](#1-go-standard-library)
- [2. IO](#2-io)
  - [2.1. MultiWriter](#21-multiwriter)
  - [2.2. io.Writer](#22-iowriter)
  - [2.3. io.ReadFull](#23-ioreadfull)
- [3. log](#3-log)
- [4. strconv](#4-strconv)
- [5. encoding](#5-encoding)
  - [5.1. json](#51-json)
  - [5.2. Marshal](#52-marshal)
  - [5.3. http](#53-http)
- [6. Context](#6-context)
- [7. sql](#7-sql)
- [8. References](#8-references)

<!-- /TOC -->

# 1. GO Standard library
Go 语言的标准库（通常被称为语言自带的电池），提供了清晰的构建模块和公共接口，包含 I/O 操作、文本处理、图像、密码学、网络和分布式应用程序等，并支持许多标准化的文件格式和编解码协议。

学习标准库达到三个层面：
1. 会使用标准库提供的 API 接口。用这些接口熟练的去写业务代码。
2. 熟悉标准库的源码，理解库作者设计的思路和优秀的编码习惯。
3. 在第二步的基础上，自己去扩充标准库，自己尝试去写、封装一些库，然后开源，让更多的人了解到，去使用自己写的库，在别人用的过程中加深理解。

优质的内容
1. 源码结构清晰，架构、布局合理。
2. 完整的文档。
3. 社区广泛。


# 2. IO


## 2.1. MultiWriter
```go
func MultiWriter(writers ...Writer) Writer
```

MultiWriter 函数是一个变参函数，可以接受任意个实现了 io.Writer 接口的值。这个函数会返回一个 io.Writer 值，这个值会把所有传入的 io.Writer 的值绑在一起。当对这个返回值进行写入时，会向所有绑在一起的 io.Writer 值做写入。

## 2.2. io.Writer

`io.Writer` 是 Go 语言标准库中的接口，用于写入数据。它定义了一个用于写入字节的通用接口，可以在不同的数据源和目标上使用。以下是 `io.Writer` 的基本用法和示例：

1. 导入标准库：

```
goCopy codeimport "io"
```

1. 使用 `io.Writer` 接口的类型进行写入。最常见的类型是 `os.File` 和 `bytes.Buffer`，但你也可以自定义实现 `io.Writer` 接口的类型。
2. 使用 `Write` 方法来写入数据。`Write` 方法的签名如下：

```
func (w WriterType) Write(p []byte) (n int, err error)
```

其中 `WriterType` 是实现了 `io.Writer` 接口的类型，`p` 是要写入的字节切片，`n` 是写入的字节数，`err` 是可能发生的错误。

以下是一些基本示例：

**示例 1: 写入到文件**

```go
package main

import (
    "io"
    "os"
)

func main() {
    // 打开文件进行写入
    file, err := os.Create("example.txt")
    if err != nil {
        panic(err)
    }
    defer file.Close()

    // 创建一个 io.Writer
    writer := io.Writer(file)

    // 写入数据
    data := []byte("Hello, World!\n")
    n, err := writer.Write(data)
    if err != nil {
        panic(err)
    }

    // 打印写入的字节数
    println(n) // 输出：13
}
```

**示例 2: 使用 bytes.Buffer**

```go
package main

import (
    "bytes"
    "io"
    "os"
)

func main() {
    // 创建一个 bytes.Buffer 作为 io.Writer
    var buf bytes.Buffer

    // 写入数据
    data := []byte("Hello, World!\n")
    n, err := buf.Write(data)
    if err != nil {
        panic(err)
    }

    // 打印写入的字节数
    println(n) // 输出：13

    // 将数据写入文件
    file, err := os.Create("example.txt")
    if err != nil {
        panic(err)
    }
    defer file.Close()

    _, err = io.Copy(file, &buf)
    if err != nil {
        panic(err)
    }
}
```

这些示例演示了如何使用 `io.Writer` 接口来写入数据到不同的目标中，包括文件和内存缓冲区。无论目标是什么，你都可以使用相同的 `Write` 方法来写入数据。在实际编程中，你可以根据需要选择合适的 `io.Writer` 实现。

## 2.3. io.ReadFull

`io.ReadFull` 是 Go 语言标准库中的一个函数，它的作用是从输入流中读取指定数量的字节，直到读取到足够数量的字节或者发生错误为止。通常，它用于确保从输入流中读取到指定数量的字节，即使输入流中的数据不够也会一直尝试读取，直到达到指定数量或者发生错误。

`io.ReadFull` 的函数签名如下：

```
func ReadFull(r Reader, buf []byte) (n int, err error)
```

其中：

- `r` 是一个实现了 `io.Reader` 接口的对象，表示输入流。
- `buf` 是一个字节数组，用来存储读取到的数据。
- 返回值 `n` 表示实际读取到的字节数。
- 返回值 `err` 表示读取过程中是否发生了错误，如果成功读取到了指定数量的字节，`err` 将为 `nil`，否则将包含一个描述错误的值。

下面是一个示例，演示如何使用 `io.ReadFull` 从输入流中读取指定数量的字节：

```go
package main

import (
	"fmt"
	"io"
	"os"
)

func main() {
	// 打开一个文件作为输入流
	file, err := os.Open("example.txt")
	if err != nil {
		fmt.Println("Error:", err)
		return
	}
	defer file.Close()

	// 读取 10 个字节的数据
	data := make([]byte, 10)
	n, err := io.ReadFull(file, data)
	if err != nil {
		fmt.Println("Error:", err)
		return
	}

	fmt.Printf("Read %d bytes: %s\n", n, data)
}
```

在上述示例中，`io.ReadFull` 从文件中读取 10 个字节的数据，如果文件中的数据不足 10 个字节，它将返回一个错误。


# 3. log
log 包实现了简单的日志服务。本包定义了 Logger 类型，该类型提供了一些格式化输出的方法。本包也提供了一个预定义的 “标准”Logger，可以通过辅助函数 Print[f|ln]、Fatal[f|ln] 和 Panic[f|ln]访问，比手工创建一个 Logger 对象更容易使用。Logger 会打印每条日志信息的日期、时间，默认输出到标准错误。Fatal 系列函数会在写入日志信息后调用 os.Exit(1)。Panic 系列函数会在写入日志信息后 panic。

- Fatal 系列函数用来写日志消息，然后使用 os.Exit(1) 终止程序。
- Panic 系列函数用来写日志消息，然后触发一个 panic。除非程序执行 recover 函数，否则会导致程序打印调用栈后终止。
- Print 系列函数是写日志消息的标准方法。

log 包的日志记录器是多 goroutine 安全的。这意味着在多个 goroutine 可以同时调用来自同一个日志记录器的这些函数，而不 会有彼此间的写冲突。标准日志记录器具有这一性质，用户定制的日志记录器也应该满足这一性质。

logger 类实现的所有方法
```go
func (l *Logger) Fatal(v ...interface{})
func (l *Logger) Fatalf(format string, v ...interface{})
func (l *Logger) Fatalln(v ...interface{})
func (l *Logger) Flags() int
func (l *Logger) Output(calldepth int, s string) error
func (l *Logger) Panic(v ...interface{})
func (l *Logger) Panicf(format string, v ...interface{})
func (l *Logger) Panicln(v ...interface{})
func (l *Logger) Prefix() string
func (l *Logger) Print(v ...interface{})
func (l *Logger) Printf(format string, v ...interface{})
func (l *Logger) Println(v ...interface{})
func (l *Logger) SetFlags(flag int)
func (l *Logger) SetPrefix(prefix string)
```

Logger 类型的函数

```go
func New(out io.Writer, prefix string, flag int) *Logger {
	return &Logger{out: out, prefix: prefix, flag: flag}
}

// 功能
创建并正确初始化一个 Logger 类型的值，返回新创建值的地址

// 参数说明
out: 指定了日志要写到的目的地；这个参数传入的值必须实现了 io.Writer 接口
prefix: 每行日志开头的前缀
flag: 定义日志包含了哪些属性（时间、文件等）
```

# 4. strconv
```
import "strconv"
```

strconv 包实现了基本数据类型和其字符串表示的相互转换。


# 5. encoding


## 5.1. json
Go 标准库中的 `encoding/json` 包提供了对 JSON（JavaScript Object Notation）数据的编码和解码功能。它允许将 Go 语言中的数据结构转换为 JSON 格式的字符串，并且可以将 JSON 格式的字符串解析为 Go 语言的数据结构。

`encoding/json` 包的主要作用如下：
1. JSON 序列化：`encoding/json` 包可以将 Go 语言的数据结构（如结构体 struct、切片 slice、映射 map 等）转换为 JSON 格式的字符串。这个过程被称为 JSON 序列化或编码。序列化过程将数据转换为可传输或存储的字符串形式，以便在网络传输或数据持久化时使用。
2. JSON 反序列化：`encoding/json` 包可以将 JSON 格式的字符串解析为 Go 语言的数据结构。这个过程被称为 JSON 反序列化或解码。反序列化过程将字符串解析为对应的数据结构，以便在程序中进行进一步的处理和使用。
3. 自定义 JSON 标签：`encoding/json` 包通过使用结构体字段上的标签（tag）来控制 JSON 编码和解码的行为。可以使用 `json:"fieldname"` 标签来指定字段在 JSON 中的名称，也可以使用其他标签选项来自定义编码和解码的行为。
4. 错误处理：`encoding/json` 包提供了处理 JSON 解析过程中可能发生的错误的机制。例如，如果 JSON 格式与 Go 数据结构不匹配，或者 JSON 中的值无法正确解析为 Go 类型，`encoding/json` 包会返回相应的错误信息。
5. 编码器和解码器：`encoding/json` 包提供了 `Encoder` 和 `Decoder` 类型，它们封装了 JSON 编码和解码的功能。这些类型提供了更高级别的 API，可以方便地进行 JSON 数据的流式编码和解码。



## 5.2. Marshal
Go 语言中，Marshal（编组）是指将数据结构或对象转换为字节流或字符串的过程。Marshal 操作通常用于将 Go 语言中的数据结构序列化为可传输或持久化的格式，例如 JSON、XML、Protobuf 等。

在 Go 语言中，Marshal 操作可以通过标准库中的 `encoding/json`、`encoding/xml ` 等包实现。这些包提供了 Marshal 函数，可以将 Go 语言中的数据结构转换为对应格式的字符串或字节流。net




## 5.3. http
- 一文说透 Go 语言 HTTP 标准库：https://www.luozhiyun.com/archives/561



# 6. Context

- Go标准库Context: https://www.liwenzhou.com/posts/Go/context/

  

# 7. sql

- [在 Go 中如何使用 database/sql 来操作数据库](https://jianghushinian.cn/2023/06/05/how-to-use-database-sql-to-operate-database-in-go)
- [Go packages database](https://pkg.go.dev/database/sql)



# 8. References
- Go Standard library: https://pkg.go.dev/std
- Go 语言中文网：https://studygolang.com/pkgdoc
- Mastering GO 中文翻译：https://wskdsgcf.gitbook.io/mastering-go-zh-cn/
- Go语言标准库 Example: https://books.studygolang.com/The-Golang-Standard-Library-by-Example/

