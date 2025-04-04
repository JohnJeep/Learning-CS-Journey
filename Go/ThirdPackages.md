<!--
 * @Author: JohnJeep
 * @Date: 2023-06-13 11:19:29
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-04-04 19:17:39
 * @Description: Golang 第三方包用法学习
 * Copyright (c) 2023 by John Jeep, All Rights Reserved. 
-->

# 1. Kratos


## 1.1. 简介
Kratos 一套轻量级 Go 微服务框架，包含大量微服务相关框架及工具。


## 1.2. 用法


## 1.3. References
- 官网: https://go-kratos.dev/
- Github: https://github.com/go-kratos/kratos


# 2. Go zero


## 2.1. 简介
zerolog 是一个高性能、零内存分配的 Go 日志库，结构化日志记录，即日志输出打印的格式为 JSON。


## 2.2. 用法
在大型工程中使用zerolog库进行日志记录可以按照以下步骤进行：
1. 引入zerolog库：在代码中导入zerolog库，使用类似于`import "github.com/rs/zerolog/log"`的语句将zerolog库引入到你的工程中。
2. 配置zerolog：根据你的需求对zerolog进行配置，例如设置日志级别、输出目标（如文件、标准输出等）、格式化选项等。你可以使用zerolog库提供的方法来进行配置，比如`zerolog.Level`、`zerolog.Output`等。
3. 创建Logger实例：使用zerolog提供的方法创建一个Logger实例，通常可以使用`zerolog.New()`来创建一个默认配置的Logger。你也可以根据需要进行自定义配置，比如设置输出格式、添加字段等。
4. 记录日志：使用Logger实例记录日志。zerolog库提供了多个级别的日志记录方法，如`Info()`、`Error()`、`Debug()`等。你可以根据需要选择适当的级别，并使用提供的方法记录日志消息。
5. 格式化日志消息：使用zerolog库提供的方法来格式化日志消息。你可以使用`Msg()`方法来记录简单的文本消息，或者使用`Interface()`、`Str()`等方法来记录结构化的日志消息。
6. 添加字段和上下文：zerolog库支持在日志消息中添加字段和上下文信息，以提供更多的上下文和可查询性。你可以使用`WithXXX()`系列方法添加字段，如`WithInt()`、`WithString()`等，或者使用`Context()`方法添加上下文信息。
7. 输出日志：使用Logger实例的输出方法将日志信息输出到目标位置。你可以使用`Log()`方法将日志输出到默认目标（通常是标准输出），或者使用`Output()`方法将日志输出到自定义的目标，如文件。

```go
package main

import (
	"os"

	"github.com/rs/zerolog"
	"github.com/rs/zerolog/log"
)

func main() {
	// 配置zerolog
	zerolog.TimeFieldFormat = zerolog.TimeFormatUnix
	logLevel := zerolog.InfoLevel
	logFormat := "2006-01-02 15:04:05.000"

	// 创建Logger实例
	logger := log.Output(zerolog.ConsoleWriter{Out: os.Stdout}).Level(logLevel).With().Timestamp().Logger().Format(logFormat)

	// 记录日志
	logger.Info().Str("event", "start").Msg("Application started")
	logger.Warn().Str("event", "warning").Msg("Something unexpected happened")
	logger.Error().Str("event", "error").Msg("An error occurred")

	// 添加字段和上下文
	logger = logger.With().Str("userID", "123").Logger()
	logger.Info().Msg("User logged in")

	// 输出日志
	logger.Info().Msg("Logging complete")
}
```


## 2.3. FAQ
在 ZeroLog 日志库中，`Err()` 和 `Error()` 方法用于记录错误信息，但它们在使用方式和输出内容上有所区别。

1. `Err()` 方法：

   `Err()` 方法是 `zerolog.Event` 结构体的方法之一，用于将错误作为日志事件的一部分记录下来。它接收一个 `error` 类型的参数，并将其添加到日志事件中。

   示例：

   ```go
   log.Err(err).Msg("An error occurred")
   ```

   在日志输出中，`Err()` 方法将错误对象记录为事件的一个字段，并将其键设置为 `"error"`。

   输出示例：

   ```go
   {"level":"error","error":"error message","message":"An error occurred"}
   ```

   `Err()` 方法可以将错误与其他日志信息一起记录，但它不会触发错误的传播或导致程序停止。

2. `Error()` 方法：

   `Error()` 方法是 ZeroLog 日志库的顶级函数之一，用于将错误信息作为独立的日志消息记录下来。它接收一个 `error` 类型的参数，并将其作为单独的日志消息输出。

   示例：

   ```go
   log.Error(err).Msg("An error occurred")
   ```

   在日志输出中，`Error()` 方法会创建一个独立的日志消息，其中包含错误信息。

   输出示例：

   ```go
   An error occurred: error message
   ```

   `Error()` 方法会将错误信息作为单独的日志消息输出，并可选择性地添加其他的日志内容。

总结：

- `Err()` 方法将错误作为事件字段记录，并可以与其他日志信息一起输出。
- `Error()` 方法将错误信息作为独立的日志消息输出，不会与其他日志内容混合在一起。


## 2.4. References
- Go packages: https://pkg.go.dev/github.com/rs/zerolog
- Github: https://github.com/rs/zerolog
- Better Stack 教程：https://betterstack.com/community/guides/logging/zerolog/ 


文档页面排版很好看，图标配色好看，很值得自己学习！！！

https://learning-cloud-native-go.github.io/




# 3. wire


## 3.1. 简介
[wire](https://github.com/google/wire) 是 google 开源用 Go 语言写的用于编译时依赖注入的（dependency injection）代码生成工具。它能够根据你的代码，生成相应的依赖注入 go 代码。

注：依赖注入的工具还有用反射实现的。


## 3.2. 用法
Wire通过读取Go代码中的注释和类型信息，生成依赖注入相关的代码。这些生成的代码包括初始化函数、依赖注入容器和相应的依赖关系。生成的代码可以帮助开发人员更容易地管理和解决复杂的依赖关系。

- `wire.NewSet()`: 将多个 provides 放到一个 set 集合中。
- `Injector`:  注入器（Injector）是通过编写一个函数声明来声明的，声明的函数体内是对 `wire.build` 的调用。



## 3.3. Reference
- Go packages: https://pkg.go.dev/github.com/google/wire 
- Github: https://github.com/google/wire
- Compile-time Dependency Injection With Go Cloud's Wire: https://go.dev/blog/wire
- Go工程化 - 依赖注入: https://go-kratos.dev/blog/go-project-wire/
- Compile-time Dependency Injection With Go Cloud's Wire: https://go.dev/blog/wire




# 4. Cobra


## 4.1. 简介
Go Cobra 是一个开源的用 Go 语言实现的命令行工具（库），被广泛用于构建命令行工具和 CLI 应用程序。它提供了一组简单且一致的API，可以帮助开发者轻松构建具有命令、子命令、标志、参数和帮助文档的命令行工具。

Cobra提供了以下主要功能和特点：

1. 命令和子命令：Cobra允许定义多个命令和子命令，通过层级结构组织命令，并支持嵌套和嵌入式子命令。
2. 标志和参数：Cobra支持在命令中定义标志（flags）和参数（arguments），可以用于接收和解析命令行输入。
3. 自动生成帮助文档：Cobra能够自动生成丰富的帮助文档，包括命令、子命令、标志、参数以及自定义用法说明。
4. 灵活的命令行解析：Cobra提供了灵活的命令行解析功能，可以轻松处理各种命令行输入情况。
5. 插件系统：Cobra支持插件系统，可以通过插件扩展和定制 CLI 应用程序的功能。

官方解释

> Cobra is built on a structure of commands, arguments & flags.
>
> **Commands** represent actions, **Args** are things and **Flags** are modifiers for those actions.


## 4.2. 用法
官方推荐命令格式

```
./APPNAME COMMAND ARG --FLAG

// 示例
hugo server --port=1313

// 解释
appName: hugo
command: server
--Flag: --port=1313
```


## 4.3. References
- Go 官网包：https://pkg.go.dev/github.com/spf13/cobra
- 官网：https://cobra.dev/


# 5. viper
Viper 是适用于 Go 应用程序的完整配置解决方案。它被设计用于在应用程序中工作，并且可以处理所有类型的配置需求和格式。

它支持以下特性：
- 设置默认值
- 从`JSON`、`TOML`、`YAML`、`HCL`、`envfile`和`Java properties`格式的配置文件读取配置信息
- 实时监控和重新读取配置文件（可选）
- 从环境变量中读取
- 从远程配置系统（etcd或Consul）读取并监控配置变化
- 从命令行参数读取配置
- 从buffer读取配置
- 显式配置值


## 5.1. 为什么选择Viper?
在构建现代应用程序时，你无需担心配置文件格式；你想要专注于构建出色的软件。Viper的出现就是为了在这方面帮助你的。

Viper能够为你执行下列操作：

1. 查找、加载和反序列化`JSON`、`TOML`、`YAML`、`HCL`、`INI`、`envfile`和`Java properties`格式的配置文件。
2. 提供一种机制为你的不同配置选项设置默认值。
3. 提供一种机制来通过命令行参数覆盖指定选项的值。
4. 提供别名系统，以便在不破坏现有代码的情况下轻松重命名参数。
5. 当用户提供了与默认值相同的命令行或配置文件时，可以很容易地分辨出它们之间的区别。

Viper会按照下面的优先级。每个项目的优先级都高于它下面的项目:

- 显示调用`Set`设置值
- 命令行参数（flag）
- 环境变量
- 配置文件
- key/value存储
- 默认值

**重要：** 目前 Viper 配置的键（Key）是大小写不敏感的。目前正在讨论是否将这一选项设为可选。


## 5.2. References

- Github：https://github.com/spf13/viper
- Go语言配置管理神器——Viper中文教程：https://www.liwenzhou.com/posts/Go/viper/


# 6. Go-toml


## 6.1. 简介
Go-toml 是一个操作 TOML 格式的 Go library。


## 6.2. References

Github: https://github.com/pelletier/go-toml

Go packages: https://pkg.go.dev/github.com/pelletier/go-toml


# 7. GORM


## 7.1. 简介
GORM是 Golang 目前比较热门的数据库 ORM 操作库，对开发者也比较友好，使用非常方便简单，使用上主要就是把struct类型和数据库表记录进行映射，操作数据库的时候不需要直接手写Sql代码，这里主要介绍 **MySQL**数据库。


## 7.2. References
- 官网: https://gorm.io/
- Github: https://github.com/go-gorm/gorm
- Go packages: https://pkg.go.dev/gorm.io/gorm


# 8. sarama


## 8.1. References
- Github: https://github.com/Shopify/sarama



# 9. Kafka-go
和sarama一样，segmentio/kafka-go也是一个纯go实现的kafka client，并且在很多公司的生产环境经历过考验，segmentio/kafka-go提供低级conn api和高级api(reader和writer)，以writer为例，相对低级api，它是并发safe的，还提供连接保持和重试，无需开发者自己实现，另外writer还支持sync和async写、带context.Context的超时写等。

不过Writer的sync模式写十分慢，1秒钟才几十条，但async模式就飞快了！

不过和confluent-kafka-go一样，segmentio/kafka-go 也没有像 sarama 那样提供 mock 测试包，我们需要自己建立环境测试。kafka-go 官方的建议时：**在本地启动一个kafka服务，然后运行测试**。在轻量级容器十分流行的时代，**是否需要 mock 还真是一件值得思考的事情**。



## 9.1. 对比
Sarama和segmentio/kafka-go是Go语言中两个常用的Kafka客户端库，它们各有优点和缺点。以下是它们的详细比较：

Sarama: 优点：
1. 社区支持：Sarama是Go语言中最受欢迎的Kafka客户端库之一，因此具有一个活跃的社区，提供了广泛的文档、教程和示例代码。
2. 功能丰富：Sarama提供了全面的Kafka功能支持，包括生产者和消费者API、事务支持、元数据管理等。它具有灵活的配置选项，使你能够自定义与Kafka集群的交互。
3. 成熟稳定：Sarama已经存在一段时间，并且在许多生产环境中得到广泛使用，因此它被认为是一个成熟且稳定的选择。


缺点：
1. 性能：尽管Sarama提供了高性能的Kafka客户端，但一些基准测试显示，与segmentio/kafka-go相比，它的性能可能稍逊一筹。
2. 依赖性：Sarama的依赖库比较多，这可能增加了项目的复杂性和构建过程的复杂性。
3. 自定义功能限制：尽管Sarama提供了广泛的功能，但在某些特定需求下，可能需要自定义实现，这可能会受到Sarama的架构限制。


segmentio/kafka-go: 优点：
1. 简单易用：segmentio/kafka-go是一个轻量级的Kafka客户端库，它具有简单、易于使用的API。它专注于提供基本的生产者和消费者功能，适用于一些简单的Kafka应用程序。
2. 性能：segmentio/kafka-go在性能方面表现出色，一些基准测试显示它比Sarama具有更高的吞吐量和更低的延迟。
3. 低依赖性：segmentio/kafka-go的依赖库相对较少，这有助于减少项目的复杂性和构建过程的复杂性。


缺点：
1. 功能限制：相对于Sarama，segmentio/kafka-go的功能较为有限。它提供了基本的生产者和消费者功能，但在某些高级特性（如事务支持、元数据管理等）方面可能不如Sarama全面。
2. 社区支持：尽管segmentio/kafka-go在一些项目中得到广泛使用，但相比Sarama，它的社区规模较小，可能在文档、教程和示例方面略显不足。

综上所述，如果你需要一个成熟、功能丰富的Kafka客户端库，并且对社区支持和文档重视，那么Sarama可能是一个更好的选择。但如果你希望一个简单、高性能的Kafka客户端库，并且对于基本的生产者和消费者功能满足需求，那么segmentio/kafka-go可能更适合你的项目。最终的选择应该根据你的具体需求和偏好来决定。


## 9.2. Reference
- Github: https://github.com/segmentio/kafka-go
- Go packages: https://pkg.go.dev/github.com/segmentio/kafka-go
- Go社区主流Kafka客户端简要对比: https://tonybai.com/2022/03/28/the-comparison-of-the-go-community-leading-kakfa-clients/


# 10. errgroup
用于处理 goroutine 中的错误。


## 11. 用法
Group 的核心能力就在于能够并发执行多个子任务，从调用者的角度，我们只需要传入要执行的函数，签名为：`func() error`即可，非常通用。如果任务执行成功，就返回 nil，否则就返回 error，并且会 cancel 那个新的 Context。

## 11.1. References

- Go packages: https://pkg.go.dev/golang.org/x/sync/errgroup


# 12. fiber


## 12.1. 简介
**Fiber**，一个受[Express](https://github.com/expressjs/express)启发的Golang **Web框架**，建立在[Fasthttp](https://github.com/valyala/fasthttp) 的基础之上。旨在**简化**、**零内存分配**和**高性能**，以及**快速**开发。Go 速度快，占用的内存少，而且性能高，这意味着它也使得 Fiber 框架更快。


## 12.2. 用法


### 12.2.1. 🧬 内置中间件
以下为`fiber`框架的内置中间件：

| 中间件                                                       | 描述                                                     |
| :----------------------------------------------------------- | :------------------------------------------------------- |
| [basicauth](https://github.com/gofiber/fiber/tree/master/middleware/basicauth) | basicauth中间件提供HTTP基本身份验证                      |
| [compress](https://github.com/gofiber/fiber/tree/master/middleware/compress) | Fiber的压缩中间件，它支持deflate，gzip 和 brotli（默认） |
| [cache](https://github.com/gofiber/fiber/tree/master/middleware/cache) | 拦截和响应缓存                                           |
| [cors](https://github.com/gofiber/fiber/tree/master/middleware/cors) | 跨域处理                                                 |
| [csrf](https://github.com/gofiber/fiber/tree/master/middleware/csrf) | CSRF攻击防护                                             |
| [filesystem](https://github.com/gofiber/fiber/tree/master/middleware/filesystem) | Fiber的文件系统中间件                                    |
| [favicon](https://github.com/gofiber/fiber/tree/master/middleware/favicon) | favicon图标                                              |
| [limiter](https://github.com/gofiber/fiber/tree/master/middleware/limiter) | `请求频率限制`中间件，用于控制API请求频率                |
| [logger](https://github.com/gofiber/fiber/tree/master/middleware/logger) | HTTP请求与响应日志记录器                                 |
| [pprof](https://github.com/gofiber/fiber/tree/master/middleware/pprof) | pprof 中间件                                             |
| [proxy](https://github.com/gofiber/fiber/tree/master/middleware/proxy) | 请求代理                                                 |
| [requestid](https://github.com/gofiber/fiber/tree/master/middleware/requestid) | 为每个请求添加一个requestid。                            |
| [recover](https://github.com/gofiber/fiber/tree/master/middleware/recover) | `Recover`中间件将程序从`panic`状态中恢复过来             |
| [timeout](https://github.com/gofiber/fiber/tree/master/middleware/timeout) | 添加请求的最大时间，如果超时，则转发给ErrorHandler。     |



### 12.2.2. 🧬 外部中间件
有`fiber`团队维护的外部中间件

| 中间件                                            | 描述                                      |
| :------------------------------------------------ | :---------------------------------------- |
| [adaptor](https://github.com/gofiber/adaptor)     | `net/http` 与 `Fiber`请求的相互转换适配器 |
| [helmet](https://github.com/gofiber/helmet)       | 可设置各种HTTP Header来保护您的应用       |
| [jwt](https://github.com/gofiber/jwt)             | JSON Web Token (JWT) 中间件               |
| [keyauth](https://github.com/gofiber/keyauth)     | 提供基于密钥的身份验证                    |
| [rewrite](https://github.com/gofiber/rewrite)     | URL路径重写                               |
| [session](https://github.com/gofiber/session)     | Session中间件                             |
| [template](https://github.com/gofiber/template)   | 模板引擎                                  |
| [websocket](https://github.com/gofiber/websocket) | Fasthttp WebSocket 中间件                 |



https://www.bookstack.cn/read/recommend/0002-gofiber.md



## 12.3. References
- 官网: https://docs.gofiber.io/
- Go packages: https://pkg.go.dev/github.com/gofiber/fiber/v2
- Github: https://github.com/gofiber/fiber


# 13. Gin


## 13.1. 简介
流行的web框架。

基于官方 net/http 内建标准库封装的。 


## 13.2. 用法
- 路由(Route)：URL 到函数的映射。
- URL
  - 静态匹配，路径固定。
    ```go
    r.GET("index", func(ctx *gin.Context) {
      ctx.JSON(200, gin.H{
        "key": "helloword", // 返回结果为json
      })
    })
    ```
  - 动态匹配，路径参数。比如：`/user/find/:id`
    ```go
    r.POST("/user/find/:id", func(ctx *gin.Context) {
      param := ctx.Param("id")
      ctx.JSON(200, param)
    })
    ```
  - 模糊匹配，使用通配符 `*`。
    ```go
    r.PUT("/about/*path", func(ctx *gin.Context) {
    	param := ctx.Param("path")
    	ctx.JSON(200, param)
    })
    ```
- API 
  - 路由分组：Group()
  - 参数查询
    - `Query()` 查询普通的参数
    - `GetQuery()` 判断指定的参数是否存在
    - `DefaultQuery()` 指定的参数不存在，给定个默认值
    - `BindQuery()` 
    - `ShouldBindQuery()`  相比 `BindQuery()`  报错后不影响。
      
      > map 类型请求参数不支持。


## 13.3. Reference
- Github: https://github.com/gin-gonic/gin
- 官网: https://gin-gonic.com/
- Go packages: https://pkg.go.dev/github.com/gin-gonic/gin
- 官方 example: https://github.com/gin-gonic/examples
- gin框架源码解析：https://www.liwenzhou.com/posts/Go/gin-sourcecode/



# 14. go-migrate
用 Go 语言编写的数据库迁移命令行工具。


## 14.1. 为什么要使用数据库迁移工具
让代码与数据的改变都进行版本控制。


## 14.2. Reference
- Github: https://github.com/golang-migrate/migrate


# 15. Watermill


## 15.1. 简介
[watermill](https://watermill.io/)是 Go 语言的一个异步消息解决方案，它支持消息重传、保存消息，后启动的订阅者也能收到前面发布的消息。`watermill`内置了多种**订阅-发布**实现，包括`Kafka/RabbitMQ`，甚至还支持`HTTP/MySQL binlog`。当然也可以编写自己的订阅-发布实现。此外，它还提供了监控、限流等中间件。

## 15.2. 难题

发布/订阅模式一些常见的问题

- 将消息发送到订阅者管道之后就不管了，这样如果订阅者处理压力较大，会在管道中堆积太多消息，一旦订阅者异常退出，这些消息将会全部丢失！
- 若没有消息保存，如果订阅者后启动，之前发布的消息，这个订阅者是无法收到的。




## 15.3. References
- 官网：https://watermill.io/
- Github: https://github.com/ThreeDotsLabs/watermill/
- Go packages: https://pkg.go.dev/github.com/ThreeDotsLabs/watermill@v1.2.0/pubsub/gochannel
- Go 每日一库之 watermill: https://cloud.tencent.com/developer/article/1693800


# 16. Gotest


## 16.1. 简介
gotests 是一款开源的表格驱动测试函数的工具，不需要从0开始写表格驱动测试函数，工具自动生成表格驱动测试函数的框架，在每个对应的测试函数中填入相应的测试案例即可。


## 16.2. 用法
**Minimum Go version:** Go 1.6

Use [`go get`](https://golang.org/cmd/go/#hdr-Download_and_install_packages_and_dependencies) to install and update:

```sh
// 终端执行
go get -u github.com/cweill/gotests/...
```

From the commandline, `gotests` can generate Go tests for specific source files or an entire directory. By default, it prints its output to `stdout`.

```sh
$ gotests [options] PATH ...
```

Available options:

```sh
  -all                  generate tests for all functions and methods

  -excl                 regexp. generate tests for functions and methods that don't
                         match. Takes precedence over -only, -exported, and -all

  -exported             generate tests for exported functions and methods. Takes
                         precedence over -only and -all

  -i                    print test inputs in error messages

  -only                 regexp. generate tests for functions and methods that match only.
                         Takes precedence over -all

  -nosubtests           disable subtest generation when >= Go 1.7

  -parallel             enable parallel subtest generation when >= Go 1.7.

  -w                    write output to (test) files instead of stdout

  -template_dir         Path to a directory containing custom test code templates. Takes
                         precedence over -template. This can also be set via environment
                         variable GOTESTS_TEMPLATE_DIR

  -template             Specify custom test code templates, e.g. testify. This can also
                         be set via environment variable GOTESTS_TEMPLATE

  -template_params_file read external parameters to template by json with file

  -template_params      read external parameters to template by json with stdin
```


终端用 gotests 命令执行代码文件 `xxx.go`，生成对应的测试文件 `xxx_test.go`，录下如果事先存在这个文件就不再生成。

```go
gotests -all -w xxx.go
```


## 16.3. References
- Github: https://github.com/cweill/gotests



# 17. Gomock

用法：



# 18. goja

`goja` 是一个用于在Go语言中运行JavaScript代码的库。它提供了与JavaScript交互的功能，包括创建对象、调用函数、访问属性等。

## 18.1. API

- `goja.New()`

  创建了一个新的JavaScript运行环境。

  Goja 虚拟机实例本身并不是线程安全的。如果你需要在多个goroutine中使用Goja，你需要为每个goroutine创建一个新的Goja虚拟机实例。如果你尝试在多个goroutine中共享一个Goja虚拟机实例，你可能会遇到并发问题。

  例如：

  ```go
  // vm 是一个goja的虚拟机对象
  vm := goja.New()
  ```

- `vm.Get()`

  获取一个Javascript变量值：

  ```go
  value, err := vm.Get("a"){
    value, _ := value.ToInteger()
  }
  ```

- `vm.Set()`

  设置一个Javascript变量的值：

  ```go
  vm.Set("a", 88)
  vm.Set("b", "hello")
  ```

  **陷阱函数**

  "get" 陷阱函数会在尝试获取代理对象的属性时被调用。例如，如果你在 JavaScript 代码中写 `Things.property` 或 `Things['property']`，那么 "get" 陷阱函数就会被调用。

  "set" 陷阱函数会在尝试设置代理对象的属性时被调用。例如，如果你在 JavaScript 代码中写 `Things.property = value`，那么 "set" 陷阱函数就会被调用



- `SetFieldNameMapper()`

  `SetFieldNameMapper`方法接受一个字段名映射器作为参数。在这里，`goja.TagFieldNameMapper("json", true)`是一个字段名映射器函数，它使用结构体字段的`json`标签作为字段名。第二个参数`true`表示忽略未标记为`json`的字段。

  通过调用`SetFieldNameMapper`方法并传递这个字段名映射器函数作为参数，我们可以告诉goja虚拟机在处理结构体时使用这个映射器来将字段名与JSON字段进行映射。

  这个功能在处理结构体与JSON之间的转换时非常有用。例如，当我们从JSON数据中解析出一个结构体对象时，可以使用这个字段名映射器来确保字段名的一致性。这行代码是在Go语言中使用goja库时的一部分。它的作用是设置一个字段名映射器，用于将结构体字段与JSON字段进行映射。

- `NewObject()`

  用于创建一个新的JavaScript对象。

- `NewProxy()`

  用于创建一个JavaScript代理对象。代理对象允许在JavaScript代码中拦截对对象属性的访问，并在Go代码中处理这些访问。

  `NewProxy`函数接受两个参数：

  1. 要代理的对象
  2. 一个`ProxyTrapConfig`对象，其中包含了对属性访问进行拦截的回调函数。

- `ToValue()`

  Go语言的值转换为JavaScript的值。这个方法接受一个Go语言的值作为参数，并返回一个表示该值的JavaScript值。

- `RunString()`

  `vm.RunString()` 方法接受一个字符串参数，这个字符串是有效的 JavaScript 代码。这个方法会在当前的 `goja` 虚拟机环境（`vm`）中执行这段 JavaScript 代码，并返回执行结果。

  返回的结果有两个：第一个是 JavaScript 代码执行后的结果，它的类型是 `goja.Value`；第二个是一个错误信息，如果在执行 JavaScript 代码过程中发生了错误，这个错误信息就会被返回。

  例如：

  ```go
  vm := goja.New()
  v, err := vm.RunString("2 + 2")
  if err != nil {
      log.Fatal(err)
  } else {
      fmt.Println(v.Export())  // 输出：4
  }
  ```

  在这个例子中，`vm.RunString("2 + 2")` 会在 `goja` 虚拟机中执行 JavaScript 代码 `"2 + 2"`，并返回执行结果 `4`。

- `export()`

  `v.Export()` 是 Go 语言库 `goja` 中的一个方法。

  `v.Export()` 方法用于将 JavaScript 的值转换为 Go 语言的值。这个方法没有参数，返回一个 `interface{}` 类型的值，这个值是 JavaScript 值在 Go 语言中的表示。

  例如，如果 JavaScript 的值是一个数字，那么 `v.Export()` 方法会返回一个 `float64` 类型的值。如果 JavaScript 的值是一个字符串，那么 `v.Export()` 方法会返回一个 `string` 类型的值。

  在你的代码中，`v.Export()` 方法用于获取 JavaScript 代码执行后的结果，并将其转换为 Go 语言的值。例如，如果 JavaScript 代码返回一个对象，那么 `v.Export()` 方法会返回一个 `map[string]interface{}` 类型的值，这个值表示 JavaScript 对象的属性和值。

- `goja.Proxy`

  `goja.Proxy`是一个自定义的类型，用于创建一个代理对象，代理对象通常用于封装对其他对象的访问。



## 18.2. References

- Gomock 实战指南：提升 Go 代码测试质量：https://www.lixueduan.com/posts/go/gomock/
- Github gomock: https://github.com/golang/mock
- CSDN gomock: https://blog.csdn.net/shanxiaoshuai/article/details/115492171



# sqlc

## 简介

## 为什么选择sqlc

在 Go 语言中编写数据库操作代码真的非常痛苦！`database/sql`标准库提供的都是比较底层的接口。我们需要编写大量重复的代码。大量的模板代码不仅写起来烦，而且还容易出错。有时候字段类型修改了一下，可能就需要改动很多地方；添加了一个新字段，之前使用 `select *` 查询语句的地方都要修改。如果有些地方有遗漏，可能就会造成运行时`panic`。即使使用 ORM 库，这些问题也不能完全解决！这时候，`sqlc`来了！`sqlc`可以根据我们编写的 SQL 语句生成类型安全的、地道的 Go 接口代码，我们要做的只是调用这些方法。

## 用法

`sqlc.yaml` 文件

```yaml
version: "1"
packages:
  - name: "db"
    path: "./db"
    queries: "./query.sql"
    schema: "./schema.sql"
    emit_json_tags: false  #默认为false，设置该字段为true可以为生成的模型对象结构添加JSON标签 
		emit_prepared_queries: false #默认为false，设置该字段为true，会为 SQL 生成对应的prepared statement
		emit_interface: false #默认为false，设置该字段为true，会为查询结构生成一个接口。最终生成的代码会多出一个文件querier.go


version：版本；
packages：
  name：生成的包名；
  path：生成文件的路径；
  queries：查询 SQL 文件；
  schema：建表 SQL 文件。
```



## References

- Go library: https://pkg.go.dev/database/sql
- sqlc office: https://sqlc.dev/
- Github sqlc: https://github.com/sqlc-dev/sqlc
- Go 每日一库之 sqlc：https://cloud.tencent.com/developer/article/1693829

# Go-sql-driver

MySQL golang 数据库驱动。

## References

- Go library: https://github.com/go-sql-driver/mysql
- [Go mysql驱动源码分析](https://blog.csdn.net/qq_39383767/article/details/144332375)