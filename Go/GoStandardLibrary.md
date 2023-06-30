<!--

 * @Author: johnjeep
 * @Date: 2022-12-27 20:41:57
 * @LastEditors: JohnJeep
 * @LastEditTime: 2023-06-16 17:25:56
 * @Description: 
 * Copyright (c) 2022 by johnjeep, All Rights Reserved. 
-->
<!-- TOC -->

- [1. GO Standard library](#1-go-standard-library)
  - [1.1. IO](#11-io)
    - [1.1.1. MultiWriter](#111-multiwriter)
  - [1.2. log](#12-log)
  - [1.3. strconv](#13-strconv)
  - [1.4. Reference](#14-reference)

<!-- /TOC -->

# 1. GO Standard library

Go语言的标准库（通常被称为语言自带的电池），提供了清晰的构建模块和公共接口，包含I/O操作、文本处理、图像、密码学、网络和分布式应用程序等，并支持许多标准化的文件格式和编解码协议。



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



# 3. log

log包实现了简单的日志服务。本包定义了Logger类型，该类型提供了一些格式化输出的方法。本包也提供了一个预定义的“标准”Logger，可以通过辅助函数Print[f|ln]、Fatal[f|ln]和Panic[f|ln]访问，比手工创建一个Logger对象更容易使用。Logger会打印每条日志信息的日期、时间，默认输出到标准错误。Fatal系列函数会在写入日志信息后调用os.Exit(1)。Panic 系列函数会在写入日志信息后 panic。

- Fatal 系列函数用来写日志消息，然后使用 os.Exit(1)终止程序。
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

strconv包实现了基本数据类型和其字符串表示的相互转换。





# 5. encoding

## 5.1. json

Go 标准库中的`encoding/json`包提供了对JSON（JavaScript Object Notation）数据的编码和解码功能。它允许将Go语言中的数据结构转换为JSON格式的字符串，并且可以将JSON格式的字符串解析为Go语言的数据结构。

`encoding/json`包的主要作用如下：

1. JSON序列化：`encoding/json`包可以将Go语言的数据结构（如结构体 struct、切片 slice、映射 map等）转换为JSON格式的字符串。这个过程被称为JSON序列化或编码。序列化过程将数据转换为可传输或存储的字符串形式，以便在网络传输或数据持久化时使用。
2. JSON反序列化：`encoding/json`包可以将JSON格式的字符串解析为Go语言的数据结构。这个过程被称为JSON反序列化或解码。反序列化过程将字符串解析为对应的数据结构，以便在程序中进行进一步的处理和使用。
3. 自定义JSON标签：`encoding/json`包通过使用结构体字段上的标签（tag）来控制JSON编码和解码的行为。可以使用`json:"fieldname"`标签来指定字段在JSON中的名称，也可以使用其他标签选项来自定义编码和解码的行为。
4. 错误处理：`encoding/json`包提供了处理JSON解析过程中可能发生的错误的机制。例如，如果JSON格式与Go数据结构不匹配，或者JSON中的值无法正确解析为Go类型，`encoding/json`包会返回相应的错误信息。
5. 编码器和解码器：`encoding/json`包提供了`Encoder`和`Decoder`类型，它们封装了JSON编码和解码的功能。这些类型提供了更高级别的API，可以方便地进行JSON数据的流式编码和解码。



### 5.1.1. Marshal

Go语言中，Marshal（编组）是指将数据结构或对象转换为字节流或字符串的过程。Marshal操作通常用于将Go语言中的数据结构序列化为可传输或持久化的格式，例如JSON、XML、Protobuf等。

在Go语言中，Marshal操作可以通过标准库中的 `encoding/json`、`encoding/xml `等包实现。这些包提供了Marshal 函数，可以将 Go 语言中的数据结构转换为对应格式的字符串或字节流。net

## 5.2. http

一文说透 Go 语言 HTTP 标准库：https://www.luozhiyun.com/archives/561



# 6. Reference

Go Standard library: https://pkg.go.dev/std

