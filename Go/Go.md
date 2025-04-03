<!--

 * @Author: JohnJeep
 * @Date: 2020-09-05 23:49:23
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-04-03 17:54:52
 * @Description: Go 语言学习
 * Copyright (c) 2022 by John Jeep, All Rights Reserved.
-->

- [1. How to learn go(学习方法)](#1-how-to-learn-go学习方法)
- [2. Concept(基本概念)](#2-concept基本概念)
  - [2.1. 吉祥物](#21-吉祥物)
  - [2.2. 命令](#22-命令)
    - [2.2.1. go mod tidy](#221-go-mod-tidy)
    - [2.2.2. go run](#222-go-run)
    - [2.2.3. goproxy](#223-goproxy)
- [3. Golang 特点](#3-golang-特点)
  - [3.1. 拥有特性](#31-拥有特性)
  - [3.2. 不支持的特性](#32-不支持的特性)
  - [3.3. 应用方向](#33-应用方向)
- [4. Go 基础](#4-go-基础)
  - [4.1. Agent(代理)](#41-agent代理)
  - [4.2. 项目目录结构](#42-项目目录结构)
  - [4.3. Compile(编译)](#43-compile编译)
  - [4.4. key-words(关键字)](#44-key-words关键字)
  - [4.5. built-in type(内置类型)](#45-built-in-type内置类型)
    - [4.5.1. Value types(值类型)](#451-value-types值类型)
    - [4.5.2. Reference types(引用类型)](#452-reference-types引用类型)
  - [4.6. built-in function(内置函数)](#46-built-in-function内置函数)
    - [4.6.1. append](#461-append)
    - [4.6.2. copy](#462-copy)
    - [4.6.3. make](#463-make)
    - [4.6.4. new](#464-new)
  - [4.7. 内置接口](#47-内置接口)
    - [4.7.1. Error()](#471-error)
  - [4.8. declare(声明)](#48-declare声明)
    - [4.8.1. var](#481-var)
    - [4.8.2. type](#482-type)
  - [4.9. Unicode](#49-unicode)
  - [4.10. Basic Data Types(基础数据类型)](#410-basic-data-types基础数据类型)
    - [4.10.1. Constant(常量)](#4101-constant常量)
    - [4.10.2. iota](#4102-iota)
    - [4.10.3. String](#4103-string)
    - [4.10.4. any](#4104-any)
  - [4.11. Composite Types(复合类型)](#411-composite-types复合类型)
    - [4.11.1. Array](#4111-array)
    - [4.11.2. Slice](#4112-slice)
    - [4.11.3. Map](#4113-map)
    - [4.11.4. Struct](#4114-struct)
      - [4.11.4.1. unname](#41141-unname)
- [5. 语言特性](#5-语言特性)
  - [5.1. Function(函数)](#51-function函数)
    - [5.1.1. 函数声明](#511-函数声明)
    - [5.1.2. 多返回值](#512-多返回值)
    - [5.1.3. 错误](#513-错误)
    - [5.1.4. 匿名函数](#514-匿名函数)
    - [5.1.5. 可变参数](#515-可变参数)
    - [5.1.6. Panic(异常)](#516-panic异常)
    - [5.1.7. defer(延迟调用)](#517-defer延迟调用)
    - [5.1.8. Recover](#518-recover)
  - [5.2. Object Oriented Programming(面向对象)](#52-object-oriented-programming面向对象)
    - [5.2.1. Method(方法)](#521-method方法)
    - [5.2.2. Encapsulation(封装)](#522-encapsulation封装)
    - [5.2.3. Inheritance(继承)](#523-inheritance继承)
    - [5.2.4. polymorphism(多态)](#524-polymorphism多态)
    - [5.2.5. Interfaces(接口)](#525-interfaces接口)
    - [5.2.6. Type Assertions(类型断言)](#526-type-assertions类型断言)
    - [5.2.7. constructor(构造函数)](#527-constructor构造函数)
  - [5.3. 泛型](#53-泛型)
- [6. Concurrency(并发)](#6-concurrency并发)
  - [6.1. Goroutine](#61-goroutine)
    - [6.1.1. 概念](#611-概念)
    - [6.1.2. 特点](#612-特点)
    - [6.1.3. goroutine scheduler](#613-goroutine-scheduler)
  - [6.2. Channel](#62-channel)
    - [6.2.1. 声明 channel](#621-声明-channel)
    - [6.2.2. 创建 channel](#622-创建-channel)
    - [6.2.3. 关闭 channel](#623-关闭-channel)
    - [6.2.4. 遍历 channel](#624-遍历-channel)
    - [6.2.5. 注意事项](#625-注意事项)
    - [6.2.6. 应用场景](#626-应用场景)
  - [6.3. 竞争(data race)](#63-竞争data-race)
    - [6.3.1. 原子函数(atmoic)](#631-原子函数atmoic)
    - [6.3.2. Mutex](#632-mutex)
    - [6.3.3. RWMutex](#633-rwmutex)
- [7. Package(包)](#7-package包)
  - [7.1. init function](#71-init-function)
  - [7.2. 包的作用](#72-包的作用)
  - [7.3. 注意事项](#73-注意事项)
  - [7.4. 包之间调用](#74-包之间调用)
  - [7.5. 打包](#75-打包)
  - [7.6. 导入包](#76-导入包)
- [8. Reflection(反射)](#8-reflection反射)
  - [8.1. 概念](#81-概念)
  - [8.2. 重要 API](#82-重要-api)
  - [8.3. 注意事项](#83-注意事项)
- [9. Problems](#9-problems)
- [10. Garbage Collector(垃圾回收)](#10-garbage-collector垃圾回收)
- [11. References](#11-references)


# 1. How to learn go(学习方法)

- 先知道怎么做，再知道为什么？
- 对知识有一个整体的框架，然后再学习具体的内容。
- 学习一门新的语言，需要掌握语言的变量、常量、表达式、控制流和函数等基本语法，这些都是每门语言通用的特性。

GO 程序员的五个进化阶段:

- **第一个阶段 (菜逼)**: 刚刚学习了这门语言。 已经通过一些教程或者培训班了解基本的语法，可以写短的代码片段。
- **第二个阶段 (探索者)**: 可以写一个完整的程序，但不懂一些更高级的语言特征，比如 “channels”。还没有使用 GO 写一个大项目。
- **第三个阶段 (大手)**: 你能熟练的使用 Go, 能够用 GO 去解决，生产环境中一个具体和完整的问题。已经形成了一套自己的惯用法和常用代码库。在你的编码方案中 Go 是一个非常好用的工具。
- **第四阶段 (大神)**: 绝逼清楚 Go 语言的设计选择和背后的动机。能理解的简洁和可组合性哲学。
- **布道师**: 积极地与他人分享关于 Go 语言知识和你对 Go 语言的理解。在各种合适的场所发出自己的声音, 参与邮件列表、建立 QQ 群、做专题报告。成为一个布道者不见得是一个完全独立的阶段，这个角色可以在上述的任何一个阶段中。

如何快速高效率地学习 Go 语言：https://studygolang.com/articles/27295


# 2. Concept(基本概念)


## 2.1. 吉祥物

GO 语言的吉祥物：樱花鼠。


## 2.2. 命令

查看 Go 工具链支持哪些命令，命令行终端输入 `go help` 即可。

```go
$ go help
Go is a tool for managing Go source code.

Usage:

        go <command> [arguments]

The commands are:

        bug         start a bug report
        build       compile packages and dependencies
        clean       remove object files and cached files
        doc         show documentation for package or symbol
        env         print Go environment information
        fix         update packages to use new APIs
        fmt         gofmt (reformat) package sources
        generate    generate Go files by processing source
        get         add dependencies to current module and install them
        install     compile and install packages and dependencies
        list        list packages or modules
        mod         module maintenance
        work        workspace maintenance
        run         compile and run Go program
        test        test packages
        tool        run specified go tool
        version     print Go version
        vet         report likely mistakes in packages

Use "go help <command>" for more information about a command.

Additional help topics:

        buildconstraint build constraints
        buildmode       build modes
        c               calling between Go and C
        cache           build and test caching
        environment     environment variables
        filetype        file types
        go.mod          the go.mod file
        gopath          GOPATH environment variable
        gopath-get      legacy GOPATH go get
        goproxy         module proxy protocol
        importpath      import path syntax
        modules         modules, module versions, and more
        module-get      module-aware go get
        module-auth     module authentication using go.sum
        packages        package lists and patterns
        private         configuration for downloading non-public code
        testflag        testing flags
        testfunc        testing functions
        vcs             controlling version control with GOVCS

Use "go help <topic>" for more information about that topic.
```

`go get` 和 `go install` 是 Go 语言中的两个不同命令，它们的主要区别在于功能和用途。

1. `go get` 
   - 功能：用于获取和安装远程包（外部依赖）。
   - 用途：主要用于在你的代码中引入其他人或团队编写的包，以便在你的项目中使用它们。
   - 示例：`go get github.com/example/package`。此命令会下载指定的远程包并将其安装到你的 Go 语言环境中的工作目录（通常是 `$GOPATH/src` 目录）或模块缓存（Go Modules）。
2. `go install` 
   - 功能：用于编译并安装可执行文件或库。
   - 用途：主要用于构建和安装你自己的 Go 项目或库，以便在本地使用或分享给其他人。
   - 示例：`go install github.com/your-username/your-package`。此命令会编译指定的包，并将生成的可执行文件或库安装到你的 Go 语言环境中的 `bin` 目录（通常是 `$GOPATH/bin` 目录）。


总结：
- `go get` 用于获取和安装外部依赖（远程包）。
- `go install` 用于构建和安装你自己的项目或库。

需要注意的是，自Go 1.16版本开始，引入了 Go Modules 作为官方的依赖管理工具，它不再依赖 `$GOPATH` 和传统的工作目录结构。使用 Go Modules 时，`go get` 和 `go install` 的行为会有所不同。它们会直接操作项目的 `go.mod` 文件和模块缓存，而不需要使用 `$GOPATH` 或 `$GOBIN`。

Go mod（又称为 Go Modules）是 Go 语言 **1.11** 及以上版本引入的一种依赖管理工具。它旨在改善 Go 语言项目的依赖管理和版本控制。

在早期，Go 语言使用 `GOPATH` 作为项目的工作空间，并使用第三方工具（如 `dep`、`glide` 等）来管理依赖关系。然而，这种方式在多项目开发和版本管理方面存在一些问题。

Go mod 的出现解决了这些问题。它允许在项目的源码目录内直接管理依赖关系，而无需依赖于 `GOPATH`。通过使用 Go mod，开发人员可以轻松地添加、更新和删除项目所需的依赖项。

Go mod 使用一个名为 `go.mod` 的文件来**记录项目的依赖关系和版本信息**。该文件会明确列出项目所依赖的模块及其版本约束。当构建项目时，Go mod 会自动下载所需的依赖项，并确保其符合指定的版本约束。

Go mod 还支持语义版本控制，使开发人员能够定义依赖项的最小和最大版本要求。这有助于确保项目在不引入不兼容的更改的情况下，持续使用稳定的依赖项版本。

总之，Go mod 是 Go 语言中的一种依赖管理工具，它简化了项目的依赖管理和版本控制，提供了更灵活和可靠的方式来管理和使用第三方库。

`go mod tidy`: 用来通过扫描当前项目中的所有代码来添加未被记录的依赖至go.mod文 件或从go.mod文件中删除不再被使用的依赖。

`go get`: 用拉添加、升级、降级或者删除单个依赖。


### 2.2.1. go mod tidy

`go mod tidy` 是 Go 语言的一个命令，用于自动处理你的 Go 项目的依赖关系。具体来说，它会做以下两件事：
1. 删除无用的模块：`go mod tidy` 会移除 go.mod 文件中所有未被项目中的任何代码直接或间接引用的模块。
2. 添加缺失的模块：`go mod tidy` 会查找所有项目中直接或间接引用的模块，如果这些模块在 go.mod 文件中没有被列出，那么 `go mod tidy` 会将它们添加进去。

这个命令通常在以下几种情况下使用：
- 当你添加了一个新的依赖，但是忘记更新 `go.mod` 文件时。
- 当你删除了一些代码，这些代码是项目中唯一引用某个依赖的地方，你想要移除这个不再使用的依赖时。
- 当你的 `go.mod` 文件因为某些原因（比如合并冲突）变得不一致，你想要修复它时。

使用这个命令的基本格式是在你的项目目录中运行 `go mod tidy`。

### 2.2.2. go run 

`go run`：这是 Go 语言的一个命令，用于编译并运行 Go 代码。它会先编译代码，然后立即运行编译后的程序。


### 2.2.3. goproxy

下载 golang tools 比较慢，一般是下载不下来的，因为 golang 的服务器在国外，被中国大陆给屏蔽了。通常想快速的下载
golang tools，一般是设置一个国内的代理。

打开 Linux terminal，然后执行下面的命令
```shell
go env -w GO111MODULE=on
go env -w GOPROXY=https://goproxy.cn,direct
```



# 3. Golang 特点


## 3.1. 拥有特性

1. 自动立即回收。
2. 更丰富的内置类型。
3. 函数多返回值。
4. 错误处理。
5. 匿名函数和闭包。
6. 包（package）。必须恰当导入需要的包，缺少了必要的包或者导入了不需要的包，程序都无法编译通过。这项严格要求避免了程序开发过程中引入未使用的包
7. 类型和接口。方法不仅可以定义在结构体上，而且，可以定义在任何用户自定义的类型上；具体类型和抽象类型（接口）之间的关系是隐式的，所以很多类型的设计者可能并不知道该类型到底实现了哪些接口。
8. 并发编程。
9. 反射。
10. 语言交互性。
11. 只读的 UTF8 字符串
12. 静态编译，编译好后，扔服务器直接运行。


## 3.2. 不支持的特性

1. 没有隐式的数值转换。
2. 没有构造函数和析构函数。
3. 没有运算符重载。
4. 没有默认参数。
5. 没有继承、类、多态。仅仅通过组合简单的对象来构建复杂的对象。
6. 没有泛型。
7. 没有异常。
8. 没有宏。
9. 没有函数修饰。
10. 没有线程局部存储。

Go 语言有足够的类型系统以避免动态语言中那些粗心的类型错误。


## 3.3. 应用方向

- 服务器编程：处理日志、数据打包、虚拟机处理、文件系统、分布式系统、数据库代理等；
- 网络编程： Web 应用、API 应用、下载应用等；
- 内存数据库和云平台领域，目前国外很多云平台都是采用 Go 开发。



# 4. Go 基础


## 4.1. Agent(代理)
众所周知，国内网络访问国外资源经常会出现不稳定的情况。 Go 生态系统中有着许多中国 Gopher 们无法获取的模块，比如最著名的 `golang.org/x/...`。并且在中国大陆从 GitHub 获取模块的速度也有点慢。
Linux 环境下，终端直接执行下面的命令进行国内镜像加速。

```shell
# 启用 Go Modules 功能
go env -w GO111MODULE=on

# 设置七牛云镜像加速
go env -w GOPROXY=https://goproxy.cn,direct

# 设置国内的能访问到的 GOSUMDB 
go env -w GOSUMDB=sum.golang.org
```

执行完后确认一下，看环境是否设置对

```shell
go env | grep GOPROXY
GOPROXY="https://goproxy.cn"
```



## 4.2. 项目目录结构

在进行 `Go` 语言开发的时候，我们的代码总是会保存在 `$GOPATH/src` 目录下。在工程经过 `go build`、`go install` 或 `go get` 等指令后，会将下载的第三方包源代码文件放在 `$GOPATH/src` 目录下， 产生的二进制可执行文件放在 `$GOPATH/bin ` 目录下，生成的中间缓存文件会被保存在 `$GOPATH/pkg` 下。

建立 Go 的工作空间（workspace，也就是 `GOPATH` 环境变量指向的目录）

Go 代码必须在工作空间内。工作空间是一个目录，其中包含三个子目录：

- `src`：源代码目录。里面每一个子目录，就是一个包。
- `pkg`： 存放编译后生成的库文件，如 go module。
- `bin`：存放编译后生成的二进制文件（目标文件）。

自动生成 `pkg`、`bin` 目录文件，需用 `go install` 命令即可，还需在 go 的环境变量中配置 `GOBIN` 路径。


## 4.3. Compile(编译)
Go 是一门编译型语言，Go 语言的工具链将源代码及其依赖转换成计算机的机器指令。Go 语言提供的工具都通过一个单独的命令 `go` 调用，`go` 命令有一系列子命令。最简单的一个子命令就是 run。这个命令编译一个或多个以 `.go` 结尾的源文件，链接库文件，并运行最终生成的可执行文件。

```go
go run xxx.go
```

Go 编译使用 `go build`, `go install` 命令。

Go build 编译参数
```go
-w 去掉调试信息
-s 去掉符号信息
-a 强制编译所有依赖包
-race 协程竞争关系

// 示例
go build -ldflags "-s -w" -o main-ldflags main.go
```





## 4.4. key-words(关键字)

25 个关键字
```go
break    default      func    interface    select
case     defer        go      map          struct
chan     else         goto    package      switch
const    fallthrough  if      range        type
continue for          import  return       var
```


## 4.5. built-in type(内置类型)


### 4.5.1. Value types(值类型)

```go
bool
int(32 or 64), int8, int16, int32, int64
uint(32 or 64), uint8(byte), uint16, uint32, uint64
float32, float64
string
complex64, complex128
array    -- 固定长度的数组
```


### 4.5.2. Reference types(引用类型)
```go
slice          -- 切片
map            -- 映射
chan           -- 管道
interface      -- 接口
function types --函数类型
```


## 4.6. built-in function(内置函数)
Go 语言拥有一些不需要进行导入操作就可以使用的内置函数。

```go
append          -- 用来追加元素到数组、slice 中, 返回修改后的数组、slice
close           -- 主要用来关闭 channel
delete          -- 从 map 中删除 key 对应的 value
panic           -- 停止常规的 goroutine  （panic 和 recover：用来做错误处理）
recover         -- 允许程序定义 goroutine 的 panic 动作
real            -- 返回 complex 的实部   （complex、real imag：用于创建和操作复数）
imag            -- 返回 complex 的虚部
make            -- 用来分配内存，返回 Type 本身（只能应用于 slice, map, channel）
new             -- 用来分配内存，主要用来分配值类型，比如 int、struct。返回指向 Type 的指针
cap             -- capacity 是容量的意思，用于返回某个类型的最大容量（只能用于切片和 map）
copy            -- 用于复制和连接 slice，返回复制的数目
len             -- 来求长度，比如 string、array、slice、map、channel ，返回长度
print、println  -- 底层打印函数，在部署环境中建议使用 fmt 包
```

官方英文文档：https://pkg.go.dev/builtin@go1.19.4


### 4.6.1. append
函数原型

```go
func append(slice []Type, elems ...Type) []Type
```

功能：在原 slice 的末尾添加元素，返回修改后的 slice，返回值是一个新的slice

```go
s1 := []int{2, 3, 5, 7}
s1 = append(s1, 10)
```


### 4.6.2. copy
函数原型：

```go
func copy(dst, src []Type) int
```

功能：将一个 slice 复制另一个相同类型的 slice。

参数：

- copy 函数的第一个参数是要复制的目标 slice，
- 第二个参数是源 slice， 目标和源的位置顺序和 `dst = src` 赋值语句是一致的。

两个 slice 可以共享同一个底层数组， 甚至有重叠也没有问题。

返回值：返回成功复制的元素的个数， 等于两个 slice 中较小的长度， 所以我们不用担心覆盖会超出目标 slice 的范围。


### 4.6.3. make
`make` 也是用于内存分配的，区别于 `new`，它只用于 `slice`、`map` 以及 `chan` 的内存创建，而且它返回的类型就是这三个类型本身，而不是他们的指针类型，因为这三种类型就是 **引用类型 **，所以就没有必要返回他们的指针了。

用法

```go
var b map[string]int
b = make(map[string]int, 10)

// 等价于
b := make(map[string]int, 10)
```

用内置的 `make` 函数创建一个指定元素类型、 长度和容量的 `slice`。  **容量部分可以省略， 在这种情况下， 容量将等于长度。**

```go
make([]T, len)
make([]T, len, cap)
```

在底层，`make` 创建了一个匿名的数组变量， 然后返回一个 `slice`； 只有通过返回的 `slice` 才能引用底层匿名的数组变量。 在第一种语句中， `slice` 是整个数组的 `view`。 在第二个语句中， `slice` 只引用了底层数组的前 `len` 个元素， 但是容量将包含整个的数组。 额外的元素是留给未来的增长用的。


### 4.6.4. new
`new` 是 Go 语言内置的一个函数，用于内存的分配。`new` 函数用于分配内存，并返回一个指向新分配的零值对象的指针。

用法

```go
// 函数签名
func new(Type) *Type

// 示例
a := new(int)

1. Type 表示类型，new 函数只接受一个参数，这个参数是一个类型
2. *Type 表示类型指针，new 函数返回一个指向该类型内存地址的指针。
```

**new 与 make 的区别**

1. 二者都是用来做内存分配的。
2. `make` 只用于 `slice`、`map` 以及 `channel` 的初始化，返回的还是这三个引用类型本身。
3. `new` 用于类型的内存分配，并且内存对应的值为类型零值，返回的是指向类型的指针。


## 4.7. 内置接口


### 4.7.1. Error()
```go
// 只要实现了 Error() 函数，返回值为 String 的都实现了 err 接口
type error interface {
	Error()    String
}
```


## 4.8. declare(声明)
Go 语言有四种主要声明方式：

```go
var（声明变量）
const（声明常量）
type（声明类型）
func（声明函数）
```

Go 的程序是保存在多个 `.go ` 文件中，文件的第一行就是 `package XXX` 声明，用来说明该文件属于哪个包（package）。`package` 声明下来就是 `import` 声明，再下来是类型，变量，常量，函数的声明。


### 4.8.1. var
Go 变量声明以关键字 `var` 开头，变量类型放在变量的后面，行尾无需分号。

其标准声明格式如下：

```go
var 变量名 变量类型

var name string
var age int
var isOk bool
```

其中，变量类型名可以省略，就变成了隐式类型定义，因为编译器可以根据变量的值来推断其类型。

```go
var name
var age
var isOk
```

批量声明。每声明一个变量就需要写 `var` 关键字会比较繁琐，Go 语言中还支持批量变量声明：

```go
var (
    a string
    b int
    c bool
    d float32
)
```

**Go 语言在声明变量的时候，会自动对变量对应的内存区域进行初始化操作。**每个变量会被初始化成其类型的默认值，例如： 整型和浮点型变量的默认值为 0；字符串变量的默认值为空字符串；布尔型变量默认为 `false`； 切片、函数、指针变量的默认为 `nil`。

简短变量声明。用于声明和初始化局部变量。 它以 ` 名字:= 表达式 ` 形式声明变量， 变量的类型根据表达式来自动推导。

```go
a := 10 // a 为 int
b := "boy" // b 为 string
```

变量分为局部变量和全局变量。

- **局部变量**：只在 `{}` 范围内定义的变量有效，离开了此作用域就失效了。
- **全部变量**：定义在函数的外部的变量。


### 4.8.2. type
类型声明通用格式

```go
type 类型名字 底层类型
```

一个类型声明语句创建了一个新的类型名称， 和现有类型具有相同的底层结构。 新命名的类型提供了一个方法， 用来分隔不同概念的类型， 这样即使它们底层类型相同也是不兼容的。**type 声明的类型是原类型的一个别名。**

```go
type Celsius float64 // 摄氏温度
type Fahrenheit float64 // 华氏温度
```

**在 Go 语言中，使用 `type` 关键字创建新的类型是一种常见的做法，这样做有几个好处：**

1. **增强代码的可读性和可理解性**：新的类型名称可以更好地描述它的用途。在你的例子中，`ThingInnerServiceName` 比 `string` 更能表达它的含义。

2. **类型安全**：新的类型不会与它的基础类型混淆。例如，你不能将一个 `ThingInnerServiceName` 类型的值赋给一个 `string` 类型的变量，除非你进行显式的类型转换。这可以防止一些类型错误。

3. **可以添加方法**：你可以为新的类型添加方法。这是 Go 语言中的一种重要特性，可以让你创建更丰富的接口。

例如：

```go
type ThingInnerServiceName string

func (t ThingInnerServiceName) Print() {
    fmt.Println("ThingInnerServiceName is: " + string(t))
}

func main() {
    var name ThingInnerServiceName = "MyService"
    name.Print()  // 输出：ThingInnerServiceName is: MyService
}
```

在这个例子中，`ThingInnerServiceName` 类型有一个 `Print` 方法，可以打印出它的值。

注意：

> Go 编译器不会对不同类型的值做隐式转换，需要做类型转换时，要显式指定。



 

## 4.9. Unicode

在很久以前， 世界还是比较简单的， 起码计算机世界就只有一个 ASCII 字符集： 美国信息交换标准代码。 ASCII， 更准确地说是美国的 ASCII， 使用 7bit 来表示 128 个字符： 包含英文字母的大小写、 数字、 各种标点符号和设备控制符。 对于早期的计算机程序来说， 这些就足够了，但是这也导致了世界上很多其他地区的用户无法直接使用自己的符号系统。

Go 语言中的 Unicode 编码为 `rune`，即 `int32` 类型。

`unicode` 包提供了 `IsDigit`、 `IsLetter`、 `IsUpper ` 和 `IsLower` 等类似功能， 它们用于给字符分类。 每个函数有一个单一的 `rune` 类型的参数， 然后返回一个布尔值  。


## 4.10. Basic Data Types(基础数据类型)

数据类型的作用：告诉编译器变量以多大的内存去存储。


### 4.10.1. Constant(常量)

常量表达式的值在编译期计算， 而不是在运行期。 每种常量的潜在类型都是基础类型：boolean、 string 或数字。
一个常量的声明语句定义了常量的名字和变量的声明语法类似， 常量的值不可修改， 这样可以防止在运行期被意外或恶意的修改。

```go
// 定义常量
const pi = 3.14
```

注意点

- 定义后，不能再修改
- 在定义的时候必须初始化
- 常量名也通过首字母的大小写来控制常量的访问范围。

批量声明常量。除了第一个外其它的常量右边的初始化表达式都可以省略， 如果省略初始化表达式则表示使用前面常量的初始化表达式写法， 对应的常量类型也一样的。

```go
// 多个常量声明
const (
	// 同时声明多个常量时，如果省略了值则表示和上面一行的值相同
	// n1 n2 n3 的值都为 100
	n1 = 100
	n2
	n3
)
```


### 4.10.2. iota
常量声明可以使用 `iota` 常量生成器初始化， 它用于生成一组以相似规则初始化的常量， 但是不用每行都写一遍初始化表达式。 在一个 `const` 声明语句中， 在第一个声明的常量所在的行，iota 将会被置为 0， 然后在每一个有常量声明的行加一。

```go
// 常量中的 iota，go 中的常量计数器
// 位于内部第一行被置为 0，每新增一行 iota 引用次数加 1
// 常用于枚举中
const (
	t1 = iota // 0
	t2        // 1
	_         // 跳过 2
	t4        // 3
	t5 = iota // 中间插入 iota，4
	t6        // 5
)

func PrintConstIota() {
	fmt.Println(t1, t2, t4, t5, t6)
}

// 定义数量级
const (
	_  = iota
	KB = 1 <<(10 * iota)
	MB = 1 <<(10 * iota)
	GB = 1 <<(10 * iota)
	TB = 1 <<(10 * iota)
	PB = 1 <<(10 * iota)
)

func PrintConstIotaShift() {
	fmt.Println(KB, MB, GB, TB, PB)
}

// 多个 iot 定义在一行
const (
	a, b = iota + 1, iota + 2 // a=0+1, b=0+2
	c, d                      // c=1+1, c=1+2
	e, f                      // e=2+1, f=2+2
)

func PrintConstIotaMulti() {
	fmt.Println(a, b, c, d, e, f)
}
```


### 4.10.3. String

标准库中有四个包对字符串处理尤为重要： `bytes`、 `strings`、`strconv` 和 `unicode` 包。

-  `strings` 包提供了许多如字符串的查询、 替换、 比较、 截断、 拆分和合并等功能。
- `bytes` 包也提供了很多类似功能的函数， 但是针对和字符串有着相同结构的 `[]byte` 类型。 因为字符串是只读的， 因此逐步构建字符串会导致很多分配和复制。 在这种情况下， 使用 `bytes.Buffer` 类型将会更有效。
- `strconv` 包提供了布尔型、 整型数、 浮点数和对应字符串的相互转换， 还提供了双引号转义相关的转换。
- `path` 和 `path/filepath` 包提供了关于文件路径名更一般的函数操作。


Strings 包中常用 API：
```go
func Contains(s, substr string) bool
func Count(s, sep string) int
func Fields(s string) []string
func HasPrefix(s, prefix string) bool
func Index(s, sep string) int
func Join(a []string, sep string) string

//  所有函数
func Clone(s string) string
func Compare(a, b string) int
func Contains(s, substr string) bool
func ContainsAny(s, chars string) bool
func ContainsRune(s string, r rune) bool
func Count(s, substr string) int
func Cut(s, sep string) (before, after string, found bool)
func EqualFold(s, t string) bool
func Fields(s string) []string
func FieldsFunc(s string, f func(rune) bool) []string
func HasPrefix(s, prefix string) bool
func HasSuffix(s, suffix string) bool
func Index(s, substr string) int
func IndexAny(s, chars string) int
func IndexByte(s string, c byte) int
func IndexFunc(s string, f func(rune) bool) int
func IndexRune(s string, r rune) int
func Join(elems []string, sep string) string
func LastIndex(s, substr string) int
func LastIndexAny(s, chars string) int
func LastIndexByte(s string, c byte) int
func LastIndexFunc(s string, f func(rune) bool) int
func Map(mapping func(rune) rune, s string) string
func Repeat(s string, count int) string
func Replace(s, old, new string, n int) string
func ReplaceAll(s, old, new string) string
func Split(s, sep string) []string
func SplitAfter(s, sep string) []string
func SplitAfterN(s, sep string, n int) []string
func SplitN(s, sep string, n int) []string
func Title(s string) stringDEPRECATED
func ToLower(s string) string
func ToLowerSpecial(c unicode.SpecialCase, s string) string
func ToTitle(s string) string
func ToTitleSpecial(c unicode.SpecialCase, s string) string
func ToUpper(s string) string
func ToUpperSpecial(c unicode.SpecialCase, s string) string
func ToValidUTF8(s, replacement string) string
func Trim(s, cutset string) string
func TrimFunc(s string, f func(rune) bool) string
func TrimLeft(s, cutset string) string
func TrimLeftFunc(s string, f func(rune) bool) string
func TrimPrefix(s, prefix string) string
func TrimRight(s, cutset string) string
func TrimRightFunc(s string, f func(rune) bool) string
func TrimSpace(s string) string
func TrimSuffix(s, suffix string) string
```

字符串常用函数

1. 统计字符串的长度，按照 字节。

   ```go
   len(str)
   ```

2. 遍历字符串，同时处理含有中文。

   ```go
   str := "中国"
   r := []rune(str)
   ```

3. 字符串转整型

   ```go
   func Atoi(s string) (int, error){}
   ```

4. 整型转字符串

   ```go
   func Itoa(i int) string {}
   ```

5. 字符串转 []byte

   ```go
   var bytes = []byte("hello")
   ```

6. []byte 转 字符串

   ```go
   str := string([]byte{97, 98, 99})
   ```

7. 判断字符串 s 是否包含子串 substr。

   ```go
   func Contains(s, substr string) bool
   ```

8. 统计一个字符串中有几个指定的字串。

   ```go
   func Count(s, sep string) int
   ```

9. 判断两个 `utf-8` 编码字符串（将 unicode 大写、小写、标题三种格式字符视为相同）是否相同。

   ```
   func EqualFold(s, t string) bool
   ```

   注意：用 `==` 比较两个字符串是否相等，是区分大小写的。

10. 将指定的字串替换为另一个字符串，返回一个替换后的新字符串，old string 没有发生被改变。传递的是值拷贝。

    ```go
    func Replace(s, old, new string, n int) string
    ```

    n 为指定替换的个数。若 n < 0，替换所有的老串。

11. 将一个字符串，按照指定的分隔符分拆分为多个字串。
    ```go
    func Split(s, sep string) []string
    ```

12. 将字符串左右两边的空格去掉
    ```go
    func TrimSpace(s string) string
    
    // fmt.Println(strings.TrimSpace("\t\n Hello, Gophers \n\t\r\n"))
    ```

13. 将字符串左右两边指定的字符串去掉
    ```go
    func Trim(s, cutset string) string
    
    // fmt.Print(strings.Trim("¡¡¡Hello, Gophers!!!", "!¡"))
    // 输出：Hello, Gophers
    ```

14. 将字符串左边指定的字符串去掉
    ```go
    func TrimLeft(s string, cutset string) string
    ```

15. 将字符串右边指定的字符串去掉
    ```go
    func TrimRight(s string, cutset string) string
    ```

`bytes` 包中常用 API：
```go
func Contains(b, subslice []byte) bool
func Count(s, sep []byte) int
func Fields(s []byte) [][]byte
func HasPrefix(s, prefix []byte) bool
func Index(s, sep []byte) int
func Join(s [][]byte, sep []byte) []byte
```

### 4.10.4. any

`any` 是 Go built-in package 中的一种类型，表示的是空接口（`interface{}`）的别名。**空接口 ** 表示可以接受任何类型的值。

原型

```go
type any = interface{}
```


## 4.11. Composite Types(复合类型)


### 4.11.1. Array

数组是一个由固定长度的特定类型元素组成的序列， 一个数组可以由零个或多个元素组成。因为数组的长度是固定的， 因此在 Go 语言中很少直接使用数组。 和数组对应的类型是 Slice（ 切片） ， 它是可以增长和收缩动态序列， slice 功能也更灵活， 但是要理解 slice 工作原	理的话需要先理解数组。

- **Declaring an array set to its zero value**
  ```go
  // Declare an integer array of five elements.
  var array [5]int
  ```
-  **Declaring an array using an array literal**
    ```go
    // Declare an integer array of five elements.
    // Initialize each element with a specific value.
    array := [5]int{10, 20, 30, 40, 50}
    ```
- **Declaring an array with Go calculating size**
  ```go
  // Declare an integer array.
  // Initialize each element with a specific value.
  // Capacity is determined based on the number of values initialized.
  array := [...]int{10, 20, 30, 40, 50}
  ```


### 4.11.2. Slice

Slice（切片）切片是围绕动态数组的概念构建的，动态数组可以根据需要增长（grow）和收缩（shrink）。一个 `slice` 类型一般写作 `[]T`， 其中 `T` 代表 `slice` 中元素的类型。slice 的语法和数组很像， 只是没有固定长度而已。  切片是数组的一个引用，因此切片是引用类型。但自身是结构体，值拷贝传递。


切片的定义
```go
var 变量名 [] 类型

// 比如:
var str []string
var arr []int

// Create a slice of strings. Use build-in function make()
// Contains a length and capacity of 5 elements.
slice := make([]string, 5)

// Create a slice of integers.
// Contains a length of 3 and has a capacity of 5 elements.
slice := make([]int, 3, 5)

// Create a slice of integers.
// Make the length larger than the capacity.
slice := make([]int, 5, 3)
Compiler Error:
len larger than cap in make([]int)

// Create a slice of strings.
// Contains a length and capacity of 5 elements.
slice := []string{"Red", "Blue", "Green", "Yellow", "Pink"}

// Create a slice of integers.
// Contains a length and capacity of 3 elements.
slice := []int{10, 20, 30}

// Create a nil slice of integers.
var slice []int
```

一个 slice 是一个轻量级的数据结构，提供了访问数组子序列（或者全部）元素的功能，而且 `slice` 的底层确实引用一个数组对象。一个 slice 由三个部分构成：**指针（addr pointer）、长度（length）和容量（capacity）**。

- 指针指向第一个 slice 元素对应的底层数组元素的地址，要注意的是 slice 的第一个元素并不一定就是数组的第一个元素。长度对应 slice 中元素的数目。
- 长度不能超过容量。切片遍历方式和数组一样，可以用 `len()` 求长度。表示可用元素数量，读写操作不能超过该限制。
- 容量一般是从 slice 的开始位置到底层数据的结尾位置。内置的 `len()` 和 `cap()` 函数分别返回 slice 的长度和容量。
  - `cap()` 可以求出 `slice` 最大扩张容量，不能超出数组限制。`0 <= len(slice) <= len(array)`，其中 array 是 slice 引用的数组。
  - 如果 `slice == nil`，那么 len、cap 结果都等于 0。

<font color=red> 注意：</font>

- slice 之间不能比较， 因此我们不能使用 `==` 操作符来判断两个 slice 是否含有全部相等元素。 不过标准库提供了高度优化的 `bytes.Equal` 函数来判断两个字节型 slice 是否相等 ，但是对于其他类型的 slice， 我们必须自己展开每个元素进行比较
- 判断一个 slice 值是否为空，使用 `len(0) == 0` 来判断，不应该使用 `s = nil` 去判断。



### 4.11.3. Map

在 Go 语言中， 一个 map 就是一个哈希表的引用， map 类型可以写为 `map[K]V`， 其中 K 和 V 分别对应 key 和 value。 map 中所有的 key 都有相同的类型， 所有的 value 也有着相同的类型， 但是 key 和 value 之间可以是不同的数据类型。 其中 K 对应的 key 必须是支持 `==` 比较运算符的数据类型， 所以 map 可以通过测试 key 是否相等来判断是否已经存在。对于 V 对应的 value 数据类型则没有任何的限制。

用内置的 `make` 函数创建一个 map

```go
m := make(map[string]int)
```

用 map literal 方式去创建 map 是很常用的方式。下面的代码中，创建一个 map 并同时给 map 赋初值。注：赋的初值可以为空值，形如：`m := map[string]int {}`。

```go
m := map[string]int {
	"John": 10,
	"Anna": 20
}
```

 Declaring a map that stores slices of strings

```go
// Create a map using a slice of strings as the value.
dict := map[int][]string{}
```

注意：

- map 中的 key：** 切片、函数不能作为 key。**

  ```go
  // Create a map using a slice of strings as the key.
  dict := map[[]string]int{}

  // 编译器报错
  Compiler Exception:
  invalid map key type []string
  ```

- map 中没有 `cap()` 函数，只有 `len()` 函数。

- map 是无序的，无法决定返回值的顺序，每次打印的返回值的结果可能不一样。

> 读>>写时，建议用 sync.Map。写>>读时，建议用 runtime.map。读=写时，建议用 courrentMap。

### 4.11.4. Struct

结构体声明

```go
type 结构体名称 struct {
	field1 type
	field2 type
}

// 示例
type Stu struct {
    Name string
    Id int
}
```

结构体中的属性名首字母大写，外部的包可以访问结构体中的变量，若属性的首字母是小写，只能在当前的包中可以访问。

创建一个结构体后，若没有给结构体赋初值，编译器会默认给结构中的字段一个零值。布尔类型是 false，整型是 0，字符串是 `""`，指 针、slice、map 的零值都是 `nil`，即还没有分配空间；数组的默认值和它的元素类型有关。

如果结构体没有任何成员的话就是空结构体， 写作 `struct{}`， 它的大小为 0， 不包含任何信息。

#### 4.11.4.1. unname

定义：没有名字的结构体，通常只用于在代码中仅使用一次的结构类型。

```go
func showMyCar() {
    newCar := struct {
        make    string
        model   string
        mileage int
    }{
        make:    "Ford",
        model:   "Taurus",
        mileage: 200000,
    }
    fmt.Printlb(newCar.mode)
}
```

使用的好处：如果一个结构体初始化后只被使用一次，那么使用匿名结构体就会很方便，不用在程序的 package 中定义太多的结构体类型，比如在解析接口的响应到结构体后，就可以使用匿名结构体

**注意**

- 结构体数据拷贝默认是 **值拷贝** 的。
- 结构体中所有字段在内存中都是连续分配的。
- 结构体是用户单独定义的类型，结构体之间相互转换时，需要结构体完全一样，即结构体的名字、字段个数、类型。
  ```go
  // 结构体 A 可以转化为结构体 B
  type A struct {
  	Num int
  }
  
  type B type {
  	Num int
  }
  ```
- 结构体进行 type 重定义（相当于取别名），Golang 认为是新的数据类型，二者之间可以强转。
  ```go
  type Teacher struct {
  	ID   int
  	Name string
  }
  
  type YuanDing Teacher // Golang 认为 YuanDing 是一种新的数据类型
  var t1 Teacher
  var y1 YuanDing
  // t1 = y1 // error：y1 与 t1 之间不能互转
  t1 = Teacher(y1) // 强转
  
  fmt.Println("t1:", t1, "y1:", y1)
  ```
- `struct` 的每个字段上，可以写一个 `tag`，该 `tag` 可以通过 **反射机制** 获取。常见的场景就是序列化和反序列化。


`for range` 循环用于迭代 array、slice、map 和 channel，但不适用于结构体类型。



# 5. 语言特性

## 5.1. Function(函数)

### 5.1.1. 函数声明

函数声明包括函数名、 形式参数列表、 返回值列表（ 可省略） 以及函数体

```go
func name(parameter-list) (result-list) {
	body
}
```

当调用一个函数的时候， 函数的每个调用参数将会被赋值给函数内部的参数变量， 所以 **函数参数变量接收的是一个复制的副本， 并不是原始调用的变量。** 因为函数参数传递的机制导致传递大的数组类型将是低效的， 并且对数组参数的任何的修改都是发生在复制的数组上， 并不能直接修改调用时原始的数组变量。

### 5.1.2. 多返回值



### 5.1.3. 错误



### 5.1.4. 匿名函数



### 5.1.5. 可变参数

### 5.1.6. Panic(异常)

Golang 的类型系统会在编译时捕获很多错误， 但有些错误只能在运行时检查， 如数组访问越界、空指针引用等。 这些运行时错误会引起 `painc` 异常。

一般而言， 当 `panic` 异常发生时， 程序会中断运行，并立即执行在该 `goroutine` 中被延迟的函数（defer 机制）。 随后， 程序崩溃并输出日志信息。 日志信息包括 `panic value` 和函数调用的堆栈跟踪信息。 `panic value` 通常是某种错误信息。

对于每个 goroutine（协程），日志信息中都会有与之相对的， 发生 panic 时的函数调用堆栈跟踪信息。 通常， 我们不需要再次运行程序去定位问题， 日志信息已经提供了足够的诊断依据。 因此， 在我们填写问题报告时， 一般会将 panic 异常和日志信息一并记录。

错误处理策略

1. 传播错误
2. 重新尝试失败的操作。如果错误的发生是偶然性的， 或由不可预知的问题导致的。 一个明智的选择是重新尝试失败的操作。 在重试时， 我们需要限制重试的时间间隔或重试的次数， 防止无限制的重试。
3. 输出错误信息并结束程序。 需要注意的是， 这种策略只应在 main 中执行。 对库函数而言， 应仅向上传播错误， 除非该错误意味着程序内部包含不一致性， 即遇到了 bug， 才能在库函数中结束程序  。
4. 只需要输出错误信息，不需要中断程序的运行。
5. 直接忽略掉错误信息。

自定义错误

Go 语言中也支持自定义错误，使用内置的 `errors.New()` 和 内置的 `panic()` 函数即可。

- `errors.New("错误说明")`：返回一个 error 类型的值，表示一个错误。内置的 error 是接口类型。可能是 nil 或者 non-nil。 nil 表示函数运行成功， non-nil 表示失败。
-  `panic()` 函数：接收一个 `error` 类型的的变量，输出错误信息，并退出程序。因为 `panic(v interface{})` 函数的参数类型为空接口（`interface{}`），任何一个变量都可以赋值给空接口。


### 5.1.7. defer(延迟调用)

`defer` 是 Go 中的一个关键字，用于延迟调用。

为什么要使用 `defer`?

> 在函数中程序员经常要创建资源（比如：数据库连接、文件句柄、锁），在函数结束时，及时的释放申请的资源，这时就要用到 defer。

在调用普通函数或方法前加上关键字 defer， 就完成了 defer 所需要的语法。 当 defer 语句被执行时， 跟在 defer 后面的函数会被延迟执行。 直到包含该 defer 语句的函数执行完毕时，defer 后的函数才会被执行， 不论包含 defer 语句的函数是通过 `return` 正常结束， 还是由于 `panic` 导致的异常结束。 你可以在一个函数中执行多条 defer 语句， 它们的执行顺序与声明顺序相反  。

- 多个 `defer` 语句执行顺序：按 **先进后出** 的方式。

- `defer` 语句中的变量，在 `defer` 声明时就决定了。

  ```go
  func Test() {
  var n1, n2 = 10, 20
     defer fmt.Println(n1) // 10
     defer fmt.Println(n2) // 20
     n1++
     n2++
     sum := n1 + n2     // sum= 11 + 21
     defer fmt.Println(sum)
  }
  ```

- 只能放在函数或方法的内部。



### 5.1.8. Recover

`recover` 是 Go 语言内置的一个函数。如果在 `defer` 函数中调用了内置函数 `recover`， 并且定义该 `defer` 语句的函数发生了 `panic` 异常， recover 会使程序从 panic 中恢复， 并返回 panic value。 导致 panic 异常的函数不会继续运行， 但能正常返回。 在未发生 panic 时调用 recover， recover 会返回 nil。

**注意：recover 只有在 defer 调用的函数中有效。**

Golang 对错误处理的方式，引入了 `Panic`、`defer`、`recover` 机制，而不再使用传统的  `try...catch...finally` 机制去处理异常，Go 语言追求简洁优雅。

使用的方式：Go 抛出一个 `Panic` 异常，在 `defer` 中用内置函数 `recover`() 去捕获异常，然后再正常处理。


## 5.2. Object Oriented Programming(面向对象)

Golang 中没有类（class），只有结构体 struct。Golang 是基于 struct 来实现面向对象的。

Golang 面向对象编程非常简洁，去掉了传统面向对象的继承、方法重载、构造函数、析构函数、隐藏的 `this` 指针。

Golang 中仍然有面向对象编程的继承、封装、多态的特性，只是实现的方式和其它的语言不一样。比如，Golang 中的 **继承** 是通过 **匿名字段** 来实现的。

**面向对象编程的步骤：**

1. 声明结构体，确定结构体的名字。
2. 编写结构体的字段。
3. 编写结构体的方法。



### 5.2.1. Method(方法)

函数声明时，在函数名字之前放了一个变量，就是一个方法。这个变量叫接收者，可以是普通类型或者是结构体。

```go
// 函数
func functionName(参数列表)(返回值列表){

}

// 方法
func (recevier type) methodName(参数列表)(返回值列表){
	方法体
    return 返回值
}
```

Go 语言的面向对象机制与一般语言不同。它没有类层次结构，甚至可以说没有类；仅仅通过组合（而不是继承）简单的对象来构建复杂的对象。

注意

- 方法作用在指定的类型上。这个类型可以是 **结构体**，也可以 **任何用户自定义的类型**；并且，具体类型和抽象类型（接口）之间的关系是隐式的，所以很多类型的设计者可能并不知道该类型到底实现了哪些接口。

- 一般方法中的类型与结构体绑定，采用 **传指针** 的方式，这样做的效率高。

- 方法的访问范围的控制规则和函数一样：若方法名首字母大写，可在本包和其它包访问；若方法名小写，只能在本包中访问。

- 自定义实现了 `String()` 方法，用 `fmt.Printf()` 函数调用时，优先执行自己定义实现的  `String()` 方法，而不是去执行 Go 中自带的  `String()` 方法。

  ```go
  // 重写 String() 方法
  type Student struct {
  	Name  string
  	Score int
  }
  
  func (stu *Student) String() string {
  	str := fmt.Sprintf("Name=%s, Score=%d", stu.Name, stu.Score)
  	return str
  }
  ```


### 5.2.2. Encapsulation(封装)

封装就是把抽象的 **字段和对字段的操作封装在一起**，数据被保护在内部，程序的其它包只有通过授权的操作，才能对字段进行访问。

如何体现封装的特性.

1. 对结构体中的属性进行封装
2. 通过方法、包实现封装。

说明：Golang 中没有特别强调封装，对面向对象的特性做了简化。


### 5.2.3. Inheritance(继承)

为什么需要继承？

> 类似功能的代码冗余，不利于维护，不利于功能的扩展。

继承的实现

> 通过嵌入一个匿名结构体来实现继承。也就是说，在 Golang 中，如果一个 struct 嵌套了另一个匿名结构体，那么这个结构体可以直接访问匿名结构体的字段和方法，从而实现了继承特性。

基本语法

```go
type Goods struct {
	Name  string
	Price float64
}

type Books struct {
	Goods // 嵌套的匿名结构体
	Color string
}
```

注意点

- 新的结构体可以使用嵌套匿名结构体中所有的字段和方法，无论是这个匿名结构体中字段、方法的首字母是否是大小写。

- 匿名结构体字段访问可以简化结构体的名字。

  ```go
  type A struct {
      Name string
      Age int
  }

  type B struct {
      A
      Score int
  }

  // 调用
  var b1 B
  b1.Name = "Xue"  // 等价于 b1.A.Name = "Xue"
  b1.Age = 66
  b1.Score  // 调用 B 结构体中自己的字段
  ```

- 当结构体和匿名结构体有相同的字段或者方法时，编译器采用 **就近访问** 原则；若希望访问匿名结构体中的字段和方法，则要通过匿名结构体名字来区分，这时不能省略匿名结构体的名字。

  ```go
  type A struct {
      Name string
      Age int
  }

  type B struct {
      A
      Name string
  }

  // 调用
  var b1 B
  b1.Name = "Xue"  // 调用的是 B 自己的字段 Name
  b1.Age = 66      // 调用匿名结构体 A 中的字段
  b1.A.Name = "Good"  // 调用匿名结构体 A 中的字段 Name，此时结构体名 A 不能省略
  ```

- 结构体嵌入两个或者多个匿名结构体，例如，两个匿名结构体有相同的字段和方法（新的结构体本身没有同名的字段和方法），在访问时，必须明确指定匿名结构体的名字，否则编译器报错。

  ```go
  type A struct {
      Name string
      Age int
  }

  type B struct {
      Name string
      Score int
  }

  type C struct {
      A
      B
      // 没有 Name
      // name string
  }

  // 调用
  var c1 C
  c1.A.Name = "wangyi"
  c2.B.Name = "lier"
  ```

- 如果一个 struct 嵌套了一个有名的结构体，这种模式就是组合，那么在访问有名的结构体的字段或方法时，必须带上结构体的名字。

  ```go
  type A struct {
      Name string
      Age int
  }

  type B struct {
      a A
  }

  // 调用
  var b2 B
  b2.a.Name = "Mike"
  be.a.Age = 20
  ```

- 如果一个 struct 中嵌套了多个匿名结构体，那么该结构体直接可以访问匿名结构体中的字段和方法，从而实现了 **多重继承**。在 Golang 开发中，为了代码的简洁性，尽量不使用多重继承。

### 5.2.4. polymorphism(多态)

接口体现多态的两种方式

1. 多态参数
2. 多态数组

### 5.2.5. Interfaces(接口)

Go 语言的接口（interface）是一种抽象的 **类型**，interface 是一组方法（method ）的集合。接口做的事情就像是定义一个协议（规则），不关心属性，只关心方法。

**为什么要有接口？**

> 降低代码的耦合性。

接口声明的通用形式：
```go
type 接口类型名 interface{
    // 声明的方法不需要实现，在外部去实现所有的方法
	方法名 1(参数列表 1) 返回值列表 1
	方法名 2(参数列表 2) 返回值列表 2
	…
}
```

每个接口由数个方法组成。
```go
// 示例
type usb interface {
	// 两个方法
	Start()
	Stop()
}
```

注意
- 接口里的方法都没有方法体，即接口中的方法是没有实现的。接口体现了程序设计的多态和高内聚低耦合的思想。

- Go 语言推荐尽量定义小的接口，并通过接口组合的方式构建程序。尽量让一个接口中的方法数量控制在 `1-3` 个内。

  **将方法写的小优势**

  - 接口越小，抽象度越高，被接纳程度越高。
  - 易于实现和测试。
  - 指责单一，易于复合使用。

- 对于接口类型，优先以**单个单词**命令。对于拥有一个或多个方法的接口组合而成的接口，Go 语言的惯例是用 `方法名+er`这样的方式来命名。

  ```go
  type Writer interface {
  	Write(p []byte) (n int, err error)
  }
  
  type Reader interface {
  	Read(p []byte) (n, int, err error)
  }
  
  type Closer interface {
  	Close() error
  }
  
  type ReadWriteCloser interface {
  	Reader
  	Writer
  	Closer
  }
  ```

- Golang 中的接口不需要显示的实现，即不需要指定结构体或者变量具体实现了哪些接口。只需要将接口中的所有方法都实现就可以了。

- Golang 的接口里面不能有任何的变量。
  ```go
  type Say interface {
  	MyPrint() // ok
  	Name string // error
  }
  ```

- 一个接口（A）可以继承多个接口（B、C），如果要实现接口 A，也必须把接口 B、C 也实现。注意：接口 B、C 中不能有相同的方法。
  ```go
  type A interface {
      Test01()
  }
  
  type B interface {
      Test02()
  }
  
  // C 接口中继承了两个接口
  type C interface {
      Test03()
      A
      B
  }
  ```
  
- **interface 类型默认是一个指针（引用类型）**，若没有初始化 interface 就是用，那么默认值为 `nil`。

- **空接口中没有任何的方法，所有的类型都实现了空接口 **。即任何一个变量都可以赋值给空接口。（**很常用**）
  
  ```go
  type Stu struct {
  	Name string
  }
  
  // 空接口
  type T interface {}
  
  // 调用
  var stu = Stu
  var t1 T = stu   // 赋结构体类型
  
  f := 3.14
  var t2 interface{} = f // 赋基础类型
  ```
  
  尽量减少将空接口（`interface{}`）作为函数的参数类型，因为空接口类型编译时会逃过编译器的类型安全检查。需要编程者自己去检查传入参数的错误信息，并且直到运行时才能发现错误。

空接口应用

-  空接口作为函数的参数
  ```go
  // 空接口作为函数参数
  // 实现可以接收任意类型的函数参数
  func show(a interface{}) {
      fmt.Printf("type:%T value:%v\n", a, a)
  }
  ```
-  空接口作为map的值
  ```go
  // 空接口作为map值，可以保存任意值的字典
  var studentInfo = make(map[string]interface{})
  studentInfo["name"] = "李白"
  studentInfo["age"] = 18
  studentInfo["married"] = false
  fmt.Println(studentInfo)
  ```

Go 中的 interface 底层包含 2 中：eface、iface。

interface 赋值的过程，即为 iface、eface 生成的过程。如果编译阶段编译器无法确定 interface 类型（比如 :iface 入参）会通过 conv 完成打包，有可能会导致逃逸。

> 很多对 interface 类型的赋值(并非所有)，都会导致空间的分配和拷贝，这也是 Interface 函数为什么可能会导致逃逸的原因 go 这么做的主要原因：逃逸的分析位于编译阶段，对于不确定的类型在堆上分配最为合适。



### 5.2.6. Type Assertions(类型断言)

类型断言（Type Assertion）是在编程语言中用于判断和转换值类型的操作。在 Go 语言中，类型断言被用于从接口类型中提取出具体的值，并判断其是否为特定的类型。 语法上它看起来像 `x.(T)` 被称为断言类型， 这里 `x` 表示类型为`interface{}`的变量，`T` 表示断言 `x` 可能是的类型。

在 Go 中，类型断言有两种形式：

1. 单个值的类型断言：
   ```go
   value, ok := x.(T)
   ```
   这种形式的类型断言用于从接口值 `x` 中提取出具体的值，并将其转换为类型 `T`。如果 `x` 的类型可以转换为 `T`，则断言成功，`value` 将接收转换后的值，`ok` 的值为 `true`。如果断言失败，即 `x` 的类型不能转换为 `T`，则 `value` 将得到 `T` 类型的零值，`ok` 的值为 `false`。

2. 类型检查的类型断言：
   ```go
   _, ok := x.(T)
   ```
    这种形式的类型断言只关注接口值 `x` 的类型是否可以转换为类型 `T`。如果可以转换，`ok` 的值为 `true`，否则为 `false`。这种形式的类型断言通常用于 **判断某个接口值是否满足特定的接口类型**。

需要注意的是，类型断言只能应用于接口类型的值。它将接口值分为两个部分：底层的具体值和该值的类型信息。类型断言可以提取底层的具体值，并检查该值是否与目标类型兼容。

以下是一个示例，展示了在 Go 中的类型断言的用法：
```go
// interface{} 为空接口
var x interface{} = "Hello"

value, ok := x.(string)
if ok {
    fmt.Println("Value:", value)
} else {
    fmt.Println("Type assertion failed")
}
```

在上述示例中，我们将字符串 `"Hello"` 赋值给接口变量 `x`。然后，使用类型断言判断 `x` 的底层类型是否为 `string` 类型。由于 `x` 的底层类型是 `string`，类型断言成功，`value` 变量接收到转换后的字符串值，并打印输出。

类型断言在 Go 中常用于处理接口类型的值，可以根据具体的类型执行特定的操作或获取特定的属性。然而，应谨慎使用类型断言，确保断言的正确性，以避免运行时的类型错误。

另一个示例来辅助说明。
```go
package main

import "fmt"

type Point struct {
	x int
	y int
}

func main() {
	var a interface{}
	var p1 Point = Point{10, 20}
	a = p1 // ok
	var p2 Point
	// p2 = a // error
	p2 = a.(Point) // 使用类型断言
	fmt.Println(p2)
}
```

注意：
- 类型断言是一个作用在接口值上的操作。
- 接口是一般的类型，不知道具体类型，要将接口转化为具体的类型，就需要使用断言。
- 使用空接口和类型断言可能会导致运行时的类型错误，因为类型断言是在 ** 运行时 ** 进行的。因此，在使用空接口和类型断言时，需要确保值的实际类型与预期类型匹配，或者通过适当的错误处理来处理类型不匹配的情况。


### 5.2.7. constructor(构造函数)

在许多编程语言中，构造函数（constructor）是一种用于创建和初始化对象的特殊方法。它的主要目的是在对象创建时执行必要的初始化操作，并确保对象的有效状态。

然而，在 Go 语言中，并没有像其他语言中的构造函数那样的特殊语法或关键字。相反，Go 推崇简洁性和灵活性，通过 **普通的函数和结构体组合来实现对象的创建和初始化。**

尽管如此，为什么还要模拟构造函数呢？以下是一些可能的原因：

1. 初始化对象：通过模拟构造函数，可以将对象的创建和初始化过程封装在一个函数中，使代码更加清晰和可读。它可以在一个地方集中处理对象的初始化逻辑，减少重复代码。
2. 隐藏内部实现：模拟构造函数可以帮助隐藏对象的内部实现细节，将对象的创建过程与外部调用代码分离。这样，外部代码只需要关注如何使用对象，而无需了解其创建和初始化的具体细节。
3. 提供参数化初始化：构造函数的模拟可以接收参数，并根据不同的参数值创建不同的对象实例。这样可以在创建对象时根据需要进行个性化的初始化。
4. 简化对象的创建：通过构造函数的模拟，可以简化对象的创建过程，尤其是在需要进行多个初始化步骤或依赖注入时。它可以封装复杂的初始化逻辑，使代码更易于维护和扩展。

需要注意的是，尽管模拟构造函数在某些情况下可以提供便利和代码组织上的好处，但在 Go 语言中，并没有像其他语言那样的语言级别的构造函数概念。因此，它仍然只是一种模拟，并没有与语言本身的特性和语法相对应。



## 5.3. 泛型

函数的泛型参数是一种在函数定义中使用的通用类型参数，可以用于表示多种不同的类型。在 Go 语言中，从 **Go 1.18** 版本开始引入了 **泛型特性**，允许开发者定义具有泛型参数的函数、方法和数据结构。

泛型参数使用方括号（**[]**）包围，并在函数名后面紧跟泛型参数列表。泛型参数可以是任何合法的标识符，用于表示类型的占位符。通过将泛型参数放在函数签名中，可以在函数体内使用它们作为类型的抽象。

泛型参数的主要作用是增加代码的复用性和灵活性，使函数能够适用于多种类型，而不仅仅局限于特定的类型。通过使用泛型参数，可以编写更通用、更灵活的函数，减少代码的重复编写。

以下是一个示例函数，展示了如何在 Go 语言中定义带有泛型参数的函数：
```go
codefunc PrintSlice[T any](slice []T) {
    for _, element := range slice {
        fmt.Println(element)
    }
}
```

在上面的示例中，`PrintSlice` 函数定义了一个泛型参数 `T`，它接受一个切片 `slice` 作为参数。在函数体内，可以使用 `T` 来表示切片中的元素类型，实现对切片元素的打印。

通过在函数调用时指定具体的类型，可以使用 `PrintSlice` 函数打印不同类型的切片。例如：
```go
codeintSlice := []int{1, 2, 3}
PrintSlice(intSlice)

stringSlice := []string{"Hello", "World"}
PrintSlice(stringSlice)
```

在上述示例中，`PrintSlice` 函数被分别调用了两次，一次传入 `int` 类型的切片，一次传入 `string` 类型的切片。由于函数使用了泛型参数 `T`，它能够适用于不同类型的切片。

**注意：**

- 泛型参数只在函数内部有效，它们的作用范围仅限于函数体内。在函数外部是无法直接使用泛型参数的，因为泛型参数在编译时会被具体的类型替换。
- 一个泛型类型必须被实例化才能被用作值类型。


# 6. Concurrency(并发)

Go 并发采用 `goroutines` 和 `channel` 去处理并发编程。


## 6.1. Goroutine


### 6.1.1. 概念

在 Go 语言中， 每一个并发的执行单元叫作一个 `goroutine`，即协程。 设想这里的一个程序有两个函数，一个函数做计算， 另一个输出结果， 假设两个函数没有相互之间的调用关系。 一个线性的程序会先调用其中的一个函数， 然后再调用另一个。 如果程序中包含多个 `goroutine`， 对两个函数的调用则可能发生在同一时刻。

如果你使用过操作系统或者其它语言提供的线程， 那么你可以简单地把 `goroutine` 类比作一个线程， 这样你就可以写出一些正确的程序了。

当一个程序启动时， 其主函数即在一个单独的 `goroutine` 中运行， 我们叫它 `main goroutine`。 新的 `goroutine` 会用 go 语句来创建。 在语法上， go 语句是一个普通的函数或方法调用前加上关键字**go**。 Go 语句会使其语句中的函数在一个新创建的 `goroutine` 中运行， 而 go 语句本身会迅速地完成。

```go
func gt() {
	// 函数体
	...
}

// 调用
go gt() // 创建一个协程去调用函数 gt()
```

**简而言之：协程就是一个轻量级的线程（编译器做了优化），一个 Go 主线程，可以起多个协程，很轻松的开启上万个协程。**

**并发(comcurrency)**：多线程在单核上运行。从微观上看，同一时间上只有一个任务在执行，这个时间点很短，短的肉眼无法分辨。

**并行(parallelism)**：多线程在多核上运行。从微观上看，同一时间点，有多个任务在同时执行。即不同的代码片段同时在不同的物理处 理器上执行。

在很多情况下，并发的效果比并行好，因为操作系统和硬件的 总资源一般很少，但能支持系统同时做很多事情。

如果希望让 goroutine 并行，必须使用多于一个逻辑处理器。当有多个逻辑处理器时，调度器 会将 goroutine 平等分配到每个逻辑处理器上。这会让 goroutine 在不同的线程上运行。不过要想真 的实现并行的效果，用户需要让自己的程序运行在有多个物理处理器的机器上。否则，哪怕 Go 语 言运行时使用多个线程，goroutine 依然会在同一个物理处理器上并发运行，达不到并行的效果。


### 6.1.2. 特点

- 有独立的栈空间。Go 运行时默认为每个 goroutine 分配的栈空间为 2KB。
- 共享程序堆空间
- 调度由用户控制。goroutine 调度的切换也并不用陷入（trap）操作系统内核完成，代价很低。



### 6.1.3. goroutine scheduler

goroutine 调度器：将 goroutine  按照一定的算法放到 CPU 上执行的程序就叫协程调度器。




## 6.2. Channel

一个 `channel` 是一个通信机制，让一个 `goroutine` 通过  `channel` 给另一个 `goroutine` 发送信息。 每个 `channel` 都有一个特殊的类型， 也就是 `channels` 可发送数据的类型。


### 6.2.1. 声明 channel

```go
// 通用用法
var 变量名 chan type

// 示例，声明一个 int 类型的 channel
var flow chan int
```

注意点
- channel 是引用类型
- channel 必须初始化后才能写入数据，即必须使用 `make` 创建之后才能用。


### 6.2.2. 创建 channel

```go
// 发送 int 类型数据；其中 cap 表示容量，可以省略
ch := make(chan int, cap)
```

注意点：
- channel 的本质就是一个数据结构 **队列**。
- channel 本身是线程安全的，多个协程同时访问时，不需要加锁。
- channel 是有类型的，指定了管道的类型，管道中就只能存放该类型。
- channel 的容量是在 make 时就指定了，不能自动增长，超过容量时，会报错。
- 在没有使用协程的情形下，若管道中的数据已全部取出，再取就报 `deadlock` 错误。
- **同步操作**：通道上的发送和接收操作是原子性的，保证了数据的一致性和可靠性。
- **阻塞机制**：当通道为空时，接收操作会阻塞等待数据；当通道满时，发送操作会阻塞等待空间。


### 6.2.3. 关闭 channel

使用内置的函数 `close()` 可以关闭 `channel`，当管道关闭后，就不能向管道中 **写（write）数据** 了，但是仍然可以从管道中 **读（read）数据**。


### 6.2.4. 遍历 channel

channel 支持 `for ... range` 的方式进行遍历。不能用普通的 `for` 循环，因为在遍历时，管道的长度是动态变化的。

遍历时请注意下面的细节：
- 遍历时，若 channel 没有关闭，返回时则出现 `deadlock` 错误。
- 遍历时，若 channel 已经关闭，则会正常遍历数据，遍历完后，退出循环。


### 6.2.5. 注意事项

1. 默认情况下，管道是双向的。
   ```go
   // 声明双向管道
   var c1 chan int
   ```

2. 声明为只读的管道，不能往管道中写入数据。
   ```go
   // 声明只读管道
   var c2 <-chan int
   ```

3. 声明为只写的管道，不能从管道中读数据。
   ```go
   // 声明只写的管道
   var c3 chan<- int
   ```

4. 使用 `select ` 解决从管道中取（read）数据阻塞的问题。在实际的开发中，可能遇见比较复杂的需求，无法确定什么时候关闭 `channel`？若不关闭就会导致 `deadlock` 问题，这时 `select` 就派上用场了。

   语法格式：
   ```go
   select {
       case v1 := <- 管道 1:
       ...
       case v2 := <- 管道 2:
       ....
       case v3 := <- 管道 3:
       ...
       default:
       ...
   }
   
   // 示例
   intChan := make(chan int, 10) // 定义一个 int 类型的管道，其容量为 10，并创建
   stringChan := make(chan string, 10)
   select {
       case v1 := <-intchan:
      		fmt.Println(v1)
       case v2 := <-stringchan:
       	fmt.Println(v2)
       default:
       	fmt.Println("nothing...")
   }
   ```

   使用 `select` 实现多路 channel 的并发控制。

5. 协程（goroutine）中使用 `recover` ，解决协程中出现异常（`Panic`）导致程序崩溃的问题。

   一个协程出现了问题导致其它的协程不能工作，致使整个程序崩溃掉。这时我们就可以在出现问题的协程中使用 `recover` 来捕获异常（Panic），进行处理，即使这个协程发生问题，不会影响其它的协程，程序照样正常运行。


### 6.2.6. 应用场景

1. 消息收发
2. 超时机制
3. 定时器


## 6.3. 竞争(data race)

### 6.3.1. 原子函数(atmoic)

```go
func incCountAtomic() {
	defer wg111.Done()
	for i := 0; i < 2; i++ {
		atomic.AddInt32(&count, 1)
		runtime.Gosched()
	}
}
```

常见的原子函数

```bash
LoadInt64
StoreInt64
LoadInt32
StoreInt32
LoadPointer
StorePointer
```

注意：原子函数操作的数据类型都比较简单，不容易处理复杂的组合数据类型或者带有逻辑的大代码。


### 6.3.2. Mutex

Go提供了另外一个sync包，用对代码段加锁解锁的办法来解决；这段代码叫做“临界区”。临界区在同一时刻只能有一个 goroutine 访问；这段代码就是同步执行的

```go
func incCountSync() {
	defer wg111.Done()
	for i := 0; i < 2; i++ {
		mutex.Lock()
		value := count
		runtime.Gosched()
		value++
		count = value
		mutex.Unlock()
	}
}
```

注意：**锁住的代码段开销尽量小，否则肯定影响性能。**


### 6.3.3. RWMutex

读写锁：允许只读操作可以并发执行，写操作需要获得完全独享的访问权限。Golang 中读写锁的类型是 `sync.RWMutex`

```go
var mu sync.RWMutex
var balance int
func Balance() int {
	mu.RLock()
	defer mu.RUnlock()
	return balance
}
```

注意：

> 仅在绝大部分goroutine都在获取读锁并且锁竞争比较激烈的时（即：goroutine一般都需要等待后才能获取锁），RWMutex才有优势。因为RWMutex需要更复杂的内部记录工作，所以在竞争不激烈的时候他比普通互斥锁还要慢。



# 7. Package(包)

利用 Go 自带的工具，使用单个命令完成编译、测试、基准测试、代码格式化、文档以及其他诸多任务。

Go 以包的形式来管理文件和项目目录结构的。所有的 go 代码都被组织在包中。一个包其实就是包含很多 `.go` 文件的一个路径，这样就实现了代码的组织和互相隔离，阅读代码从一个包开始。要想写好 go 代码，要不断理解包和基于包多实践。


## 7.1. init function

每个包可以包含任意多个 `init` 函数，这些函数都会在程序执行开始的时候被串行调用且仅调用执行一次。所有被编译器发现的 `init` 函数都会安排在 `main` 函数之前执行。

 `init` 函数用在设置包、初始化变量或者其他要在程序运行前优先完成的引导工作。

在同一个源文件中声明的init函数将按从上到下的顺序被调用执行。 对于声明在同一个包中的 两个不同源文件中的两个init函数，Go语言白皮书推荐（但不强求）按照它们所处于的源文件的 名称的词典序列（对英文来说，即字母顺序）来调用。 所以最好不要让声明在同一个包中的两个 不同源文件中的两个init函数存在依赖关系。 

在加载一个代码包的时候，此代码包中声明的所有包级变量都将在此包中的任何一个init函数执 行之前初始化完毕。


## 7.2. 包的作用

- 区分相同名字的函数、变量等标识符。
- 当项目的文件很多时，可以很好的管理项目。
- 控制函数、变量等访问的范围。


## 7.3. 注意事项

- **一个 package 对应一个文件夹。** 文件的 `package` 名通常与文件所在的文件名字一致，一般为小写字母。
- 在 `import` 包时，路径从 `$GOPATH` 的 `src` 下开始，不用带 `src`，编译器自动从 `src` 下开始引入。
- 为了让其它的包文件，可以访问到本包的函数，该函数名的是字母必须大写，类似其它语言中的 `public`，这样才能挎包访问。若函数名的首字母是小写，函数只能在当前的包中使用，类似于其它语言的 `private`。
- 访问其它的包函数或变量时，其语法格式是：
  ```go
  包名. 函数名
  
  包名. 变量名
  ```
- 在同一个包下不能有相同的两个函数名，否则会报重复定义。


## 7.4. 包之间调用

1、不同包之间是同一级目录
- 同一个目录时，包 1 去调用 包 2，包 2 中的包名必须和包 1 中一样。
- 包 1 去调用 包 2 中的函数，包 1 中直接调用 包 2 中函数即可，不需要引用包 2 名中的包名。

2、不同包之间是不同的目录
- 一个包去调用另外一个包，先导入另一个包，函数调用格式：` 包名. 函数名 `
- 另一个包中的函数名称的首字母必须大写，若是小写字母，调用的包不能识别当前包中的函数。


## 7.5. 打包

打包基本语法
```go
package 包名
```


## 7.6. 导入包

import 语句告诉编译器到磁盘的哪里去找想要导入的包。导入包需要使用关键字 import，它会告诉编译器你想引用该位置的包内的代码。如果需要导入多个包，习惯上是将 import 语句包装在一个导入块中

每个包是由一个全局唯一的字符串所标识的导入路径定位。

导入包的基本语法
```go
import "包路径"
```

多种方式导入包
```go
// 导入单个包
import "fmt"

// 导入多个包
import (
	"fmt"
    "os"
)

// 给导入的包起一个别名
import io "fmt"
io.Println("调用包别名")

// 忽略导入的包。目的：为了调用包里面的 init 函数
import _ "fmt"
```

编译器会使用 Go 环境变量设置的路径，通过引入的相对路径来查找磁盘上的包。标准库中的包会在安装 Go 的位置找到。 Go 开发者创建的包会在 GOPATH 环境变量指定的目录里查找。GOPATH 指定的这些目录就是开发者的个人工作空间。



# 8. Reflection(反射)

反射是一个强大的编程工具，是一种程序在运行期间审视自己的能力。


## 8.1. 概念

Go 语言提供了一种机制， 能够在运行时更新变量和检查它们的值、 调用它们的方法和它们支持的内在操作， 而不需要在编译时就知道这些变量的具体类型， 这种机制被称为反射。


## 8.2. 重要 API

- `reflect.TypeOf(变量名)`：获取变量的类型，返回 `reflect.Type` 类型。
- `reflect.ValueOf(变量名)`：获取变量的值，返回 `reflect.Value` 类型，`reflect.Value` 是一个结构体类型。
  ```go
  type Value struct {
      // 内含隐藏或非导出字段
  }
  ```
- `reflect.Value.Elem()`：Elem 返回 v 持有的接口保管的值的 Value 封装，或者 v 持有的指针指向的值的 Value 封装。
  ```go
  num := 100
  rval := reflect.Value(&num) // 传入的时 num 地址
  rval.Elem().SetInt(200) // 改变传入的 int 类型的值
  fmt.Println("value": num)  // num=200
  ```

例子：利用反射去获取结构体的字段、方法、标签。


## 8.3. 注意事项

1. 基于反射的代码是比较脆弱的。 对于每一个会导致编译器报告类型错误的问题， 在反射中都有与之相对应的误用问题， 不同的是编译器会在构建时马上报告错误， 而反射则是在真正运行到的时候才会抛出 `panic` 异常， 可能是写完代码很久之后了， 而且程序也可能运行了很长的时间。
2. 即使对应类型提供了相同文档， 但是反射的操作不能做静态类型检查， 而且大量反射的代码通常难以理解。
3. 基于反射的代码通常比正常的代码运行速度慢一到两个数量级。


# 9. Problems

大规模软件开发存在的问题。

1. 程序构建慢；
2. 依赖复杂；
3. 开发人员使用编程语言的不同子集。
4. 代码理解性差；（可读性差、文档差）
5. 功能重复实现；
6. 升级更新消耗大；
7. 实现自动化工具难度高；
8. 版本问题；
9. 跨语言问题；


# 10. Garbage Collector(垃圾回收)

在 Go 语言中，内存的分配和释放是由垃圾回收器（Garbage Collector）自动处理的，而不需要显式地进行手动内存管理。

垃圾回收器会自动跟踪对象的引用和使用情况，在对象不再被引用时自动回收其所占用的内存。

开发者可以专注于业务逻辑而无需手动管理内存。通过使用 `new` 函数创建对象后，当对象不再被引用时，垃圾回收器会自动将其标记为不可达，并在适当的时候回收其占用的内存。


# 11. References

- Go 官方英文文档：https://go.dev/
- Go 官方英文标准库：https://pkg.go.dev/std
- Go 官方 Github 源码：https://github.com/golang/go
- Go 官方 Wiki：https://github.com/golang/go/wiki
- Go 官方命令文档: https://pkg.go.dev/cmd/go
- Github 英文 Go 开发者成长路线图：https://github.com/Alikhll/golang-developer-roadmap
- Go Slice Tricks Cheat Sheet: https://ueokande.github.io/go-slice-tricks/
- golang goproxy.cn: https://goproxy.cn

------------------------------------------------------------------------
Project structure
- <font color=red>Gihub golang-standards: https://github.com/golang-standards/project-layout </font>
- golang 编程规范 - 项目目录结构：https://makeoptim.com/golang/standards/project-layout/
- Go 工程化 (二) 项目目录结构 Go 工程化 (二) 项目目录结构：https://lailin.xyz/post/go-training-week4-project-layout.html



------------------------------------------------------------------------
Packages
- Package names: https://go.dev/blog/package-names
- Style guideline for Go packages: https://rakyll.org/style-packages/
- <font color=red>Github paper-code: https://github.com/danceyoung/paper-code/tree/master </font>
  对一些好的技术文章结合自己的实践经验进行翻译、举例说明等或自己的经验分享。主要包括架构设计、模式设计、模型设计、重构及源码解析等。


------------------------------------------------------------------------
Testing
- 极客兔兔 Go Test 单元测试简明教程：https://geektutu.com/post/quick-go-test.html
- Go单测从零到溜系列0—单元测试基础：https://www.liwenzhou.com/posts/Go/unit-test-0/

------------------------------------------------------------------------
Garbage
- Go 官网 A Guide to the Go Garbage Collector: https://tip.golang.org/doc/gc-guide


------------------------------------------------------------------------
Reflection
- https://halfrost.com/go_reflection



------------------------------------------------------------------------
Tutorials
- Go 教程：https://www.topgoer.com/ 非常详细，值得学习。
- Go 语言圣经中文教程：https://books.studygolang.com/gopl-zh/
- Golang 标准库中文文档：https://studygolang.com/pkgdoc
- Go 中文文档教程：https://tour.go-zh.org/list

------------------------------------------------------------------------
log
- [Go语言结构化日志：打破日志的边界，解放你的应用程序](https://devopsman.cn/archives/go-yu-yan-jie-gou-hua-ri-zhi--da-po-ri-zhi-de-bian-jie--jie-fang-ni-de-ying-yong-cheng-xu)
- slog：Go官方版结构化日志包：https://tonybai.com/2022/10/30/first-exploration-of-slog/


------------------------------------------------------------------------
Community skills sharing 
- [LeetCode-Go](https://github.com/halfrost/LeetCode-Go): GO 语言题解 LeetCode，比较全面，使用 GO 语言时值得参考。
- [Halfrost-Field 冰霜之地](https://github.com/halfrost/Halfrost-Field)：Github 上的一位作者记录了学习 GO 语言的一些方法和经验。
- [Go 语言问题集 (Go Questions)](https://www.bookstack.cn/read/qcrao-Go-Questions/README.md)：作者学习 Go 语言的笔记
- Go 语言设计与实现：https://draveness.me/golang/
  这位作者是个大牛，开源作品很多，该项目系统的讲解了 Go 语言的知识，非常值得学习。
- 雨痕笔记，Go 语言大佬：https://www.yuque.com/qyuhen/go
- 嗨客网：https://haicoder.net/golang/golang-lib.html
- learnku 社区网站：https://learnku.com/go
- 腾讯大佬个人博客：https://www.hitzhangjie.pro/blog/
- **印度小哥 mohitkhare 博客**：https://www.mohitkhare.com/blog/go-dependency-injection/
- Go 语言高性能编程：https://geektutu.com/post/high-performance-go.htm
  介绍了 Go 中一些常踩的坑和性能优化技巧。
- Go的50度灰：Golang新开发者要注意的陷阱和常见错误：https://colobu.com/2015/09/07/gotchas-and-common-mistakes-in-go-golang/
- Go总结（九）| goroutine+channel并发编程：https://chende.ren/2020/12/28140907-009-concurrency.html

------------------------------------------------------------------------
Github excellent open source project
- golang-open-source-projects: https://github.com/hackstoic/golang-open-source-projects
- awesome-go-cn: https://github.com/jobbole/awesome-go-cn
- **go-awesome**: https://github.com/shockerli/go-awesome
- **国外 awesome-go**: https://github.com/avelino/awesome-go
- **Go 夜读：**
  - **https://github.com/talkgo/read**  
  - https://github.com/talkgo/night


------------------------------------------------------------------------
Tools
- https://colobu.com/gotips/041.html 
- 技术文章摘抄：https://learn.lianglianglee.com/
