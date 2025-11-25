<!--
 * @Author: JohnJeep
 * @Date: 2025-04-14 17:20:50
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-11-20 10:53:19
 * @Description: golang 面试题
 * Copyright (c) 2025 by John Jeep, All Rights Reserved. 
-->

- [1. Golang 方向](#1-golang-方向)
  - [1.1. 核心领域应用方向](#11-核心领域应用方向)
  - [1.2. 哪些行业在用Go](#12-哪些行业在用go)
  - [1.3. 使用 go 能干什么](#13-使用-go-能干什么)
  - [1.4. 优势](#14-优势)
- [2. 面试问题](#2-面试问题)
  - [2.1. goroutine内存泄漏的情况？如何避免](#21-goroutine内存泄漏的情况如何避免)
  - [2.2. 什么是死锁？如何快速定位死锁？](#22-什么是死锁如何快速定位死锁)
  - [2.3. 讲一下协程和线程的区别？](#23-讲一下协程和线程的区别)
  - [2.4. 讲一下**golang协程是如何调度的**](#24-讲一下golang协程是如何调度的)
  - [2.5. slice和array的区别，讲一下底层的结构 ,其 len，cap，共享，扩容 是怎样的？](#25-slice和array的区别讲一下底层的结构-其-lencap共享扩容-是怎样的)
  - [2.6. channel 的用途和使用上要注意的点，底层是怎样实现的？](#26-channel-的用途和使用上要注意的点底层是怎样实现的)
  - [2.7. go底层里面有没有自动回收对象的机制？](#27-go底层里面有没有自动回收对象的机制)
  - [2.8. 项目gin框架使用了什么路由中间件](#28-项目gin框架使用了什么路由中间件)
  - [2.9. grpc、proto编写规则](#29-grpcproto编写规则)
  - [2.10. grpc 怎么连接的](#210-grpc-怎么连接的)
  - [2.11. grpc和http的区别](#211-grpc和http的区别)
  - [2.12. 协程的同步方式有哪些](#212-协程的同步方式有哪些)
  - [2.13. 通用的http请求日志打印如何封装](#213-通用的http请求日志打印如何封装)
  - [2.14. 介绍下golang 标准库的 mutex](#214-介绍下golang-标准库的-mutex)
  - [2.15. go struct 能不能比较](#215-go-struct-能不能比较)
  - [2.16. map如何顺序读取](#216-map如何顺序读取)
  - [2.17. Golang 如何性能调优](#217-golang-如何性能调优)


## 1. Golang 方向

### 1.1. 核心领域应用方向

- Web服务
- 云服务
- IT基础设施
- AI 基础设施



### 1.2. 哪些行业在用Go

- **科技(超过40%):** Google, DataDog, K8s, HashiCorp, Dropbox, Salesforce, Apple...
- **金融服务(13%):** Monzo, American Express, Mercado Libre...
- **交通与零售(10%):** Amazon, Uber, DeliveryHero, HelloFresh...
- **媒体/游戏(7%):** Netflix, Bytedance, Tencent, Reddit, Snap...

 

### 1.3. 使用 go 能干什么

1. **API/RPC服务(75%)**

2. **命令行工具(62%)**

3. 1. **Ollama**
   2. **Kserver**
   3. **LangChain Go**

4. **云服务、微服务**

 

### 1.4. 优势

- Go: 容易上手，强调快速开发（**time to value**）和可伸缩性
- **Rust:** 性能极致，适用于性能密集型、底层嵌入式开发，但**复杂性更高，开发成本和时间也更高**。

 

References

- [Is Golang Still Growing? Go Language Popularity Trends in 2024]( https://blog.jetbrains.com/research/2025/04/is-golang-still-growing-go-language-popularity-trends-in-2024/)



## 2. 面试问题

### 2.1. goroutine内存泄漏的情况？如何避免

goroutine内存泄漏基本上是因为异常导致阻塞, 可以导致阻塞的情况

1. 死锁。 goroutine 等待的锁发生了死锁情况。

2. chan没有正常被关闭,导致读取读chan的goroutine阻塞。


如何避免

1. 避免死锁。
2. 正常关闭。
3. 使用context管理goroutine，超时结束goroutine。



### 2.2. 什么是死锁？如何快速定位死锁？

**锁：两个或多个goroutine 互相等待对方释放资源，导致所有 goroutine都无法执行。**

利用 Go 运行时的死锁检测功能。Go 运行时自带死锁检测机制，在程序运行期间，若检测到死锁，就会输出详细的错误信息。



**借助** **golang** **自带的** **go tool trace** **工具。**

1. 代码里面添加 trace 追踪功能
2. 运行程序生成追踪文件 trace.out
3. 执行 go tool trace     trace.out 命令，在浏览器中打开追踪文件
4. 查看goroutine 的状态和调用堆栈，找到发生死锁的位置。

也可以借助第三方库：godeadlock，帮助检测死锁。

 

### 2.3. 讲一下协程和线程的区别？

 

### 2.4. 讲一下**golang协程是如何调度的**

 

### 2.5. slice和array的区别，讲一下底层的结构 ,其 len，cap，共享，扩容 是怎样的？

**array 本质是一个固定数组,** 内存层面就是一块固定的内存区域,不会改变, 传递的时候是拷贝一份完整数据。

slice 本质上是一个动态数组的封装，底层指向不是一个固定内存,可以重新指向新的内存,传递的时候底层指向相同的内存。



### 2.6. channel 底层是怎样实现的？使用时要注意哪些？

channel 是 golang 协程之间的数据交互的重要工具，相当于与进程内的一个消息队列。

注意点: 最重要的是 chan 的 close 处理, 不然很容易出现异常, 

1. 写数据goroutine中调用close,
2.  不要多次调用close,
3.  使用信号通知 chan close底层结构

 

### 2.7. go底层里面有没有自动回收对象的机制？

runtime.finalizer



### 2.8. 项目gin框架使用了什么路由中间件

1. **Logger 中间件
   **

   Logger 中间件用于记录每个 HTTP     请求的相关信息，如请求方法、请求路径、响应状态码、响应时间等。示例代码如下：

   ```go
   package main
   import (
       "github.com/gin-gonic/gin"
   )
   	func main() {
       r := gin.Default() // 默认使用 Logger 和 Recovery 中间件
       r.GET("/", func(c *gin.Context) {
           c.JSON(200, gin.H{
               "message": "Hello, World!",
           })
       })
       r.Run()
   }
   
   ```

2. **Recovery 中间件
   ** 

   Recovery 中间件用于捕获并恢复处理过程中发生的 panic，避免因某个请求处理时的 panic，导致整个服务崩溃。在上面使用 `gin.Default()` 创建引擎时，已经默认使用了该中间件。

3. **Gzip中间件
   **

   Gzip 中间件用于对响应内容进行 Gzip 压缩，以减少数据传输量，提高响应速度。示例代码如下：

   ```go
   package main
   import (
       "github.com/gin-gonic/gin"
       "github.com/gin-gonic/gin/middleware"
   )
   	func main() {
       r := gin.Default()
       r.Use(middleware.Gzip(middleware.DefaultCompression))
       r.GET("/", func(c *gin.Context) {
           c.JSON(200, gin.H{
               "message": "Hello, World!",
           })
       })
       r.Run()
   }
   ```

4. **自定义中间件**

   你也可以根据自己的需求创建自定义中间件。例如，创建一个简单的身份验证中间件：

   ```go
   package main
   import (
       "github.com/gin-gonic/gin"
       "net/http"
   )
   	func authMiddleware() gin.HandlerFunc {
       return func(c *gin.Context) {
           // 模拟身份验证逻辑
           token := c.GetHeader("Authorization")
           if token != "valid_token" {
               c.JSON(http.StatusUnauthorized, gin.H{
                   "message": "Unauthorized",
               })
               c.Abort()
               return
           }
           c.Next()
       }
   }
   		func main() {
       r := gin.Default()
       r.GET("/public", func(c *gin.Context) {
           c.JSON(200, gin.H{
               "message": "This is a public route",
           })
       })
       privateGroup := r.Group("/private")
       privateGroup.Use(authMiddleware())
       privateGroup.GET("/", func(c *gin.Context) {
           c.JSON(200, gin.H{
               "message": "This is a private route",
           })
       })
       r.Run()
   }
   ```

   

### 2.9. grpc、proto编写规则



### 2.10. grpc 怎么连接的



### 2.11. grpc和http的区别



### 2.12. 协程的同步方式有哪些

1. Channel：有缓冲Channel和无缓冲Channel。
2. sync.WaitGroup：等待完成。
3. sync.Mutex（互斥锁）





### 2.13. 通用的http请求日志打印如何封装



### 2.14. 介绍下golang 标准库的 mutex



### 2.15. golang 中 struct 能不能比较

在 Go 语言中，struct 能否比较取决于其字段的类型。具体情况如下：

#### 可比较的情况

**当 struct 的所有字段都是可比较类型时**，该 struct 就是可比较的：

```go
type Person struct {
    Name string
    Age  int
}

p1 := Person{"Alice", 25}
p2 := Person{"Alice", 25}
p3 := Person{"Bob", 30}

fmt.Println(p1 == p2) // true
fmt.Println(p1 == p3) // false
```

可比较的字段类型包括：
- 基本类型：`bool`, `int`, `float`, `string` 等
- 指针类型
- 数组（当元素类型可比较时）
- 其他可比较的 struct
- channel

#### 不可比较的情况

**当 struct 包含不可比较的字段时**，该 struct 就不可比较：

```go
type Container struct {
    Data []int      // 切片不可比较
    // Mutex sync.Mutex // 互斥锁不可比较
}

c1 := Container{Data: []int{1, 2, 3}}
c2 := Container{Data: []int{1, 2, 3}}

// fmt.Println(c1 == c2) // 编译错误：struct containing []int cannot be compared
```

不可比较的字段类型包括：
- 切片（slice）
- 映射（map）
- 函数（function）
- 包含不可比较字段的 struct

#### 比较方法

##### 1. 使用 `reflect.DeepEqual`

```go
import "reflect"

c1 := Container{Data: []int{1, 2, 3}}
c2 := Container{Data: []int{1, 2, 3}}

fmt.Println(reflect.DeepEqual(c1, c2)) // true
```

##### 2. 自定义比较函数

```go
func (c1 Container) Equal(c2 Container) bool {
    if len(c1.Data) != len(c2.Data) {
        return false
    }
    for i, v := range c1.Data {
        if v != c2.Data[i] {
            return false
        }
    }
    return true
}

fmt.Println(c1.Equal(c2)) // true
```

##### 3. 使用 `cmp` 包（需要安装）

```go
import "github.com/google/go-cmp/cmp"

fmt.Println(cmp.Equal(c1, c2)) // true
```

#### 总结

- **可比较**：所有字段都是可比较类型
- **不可比较**：包含任何不可比较类型的字段
- 对于不可比较的 struct，可以使用 `reflect.DeepEqual` 或自定义比较函数

在实际开发中，建议根据具体需求选择合适的比较方式，对于复杂结构推荐使用自定义比较函数以获得更好的性能和类型安全。

### 2.16. map如何顺序读取



### 2.17. Golang 如何性能调优

利用golang 自带的 pprof 工具，生成 .out 文件，在浏览器中查看具体的信息

```bash
go tool pprof http://localhost:6060/debug/pprof/profile  # CPU分析
go tool pprof http://localhost:6060/debug/pprof/heap     # 内存分析
go tool pprof -http=:8080 profile.out                    # 网页可视化
```

