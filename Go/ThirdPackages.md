# 1. 框架

## 1.1. Kratos

### 1.1.1. 简介

Kratos 一套轻量级 Go 微服务框架，包含大量微服务相关框架及工具。

### 1.1.2. 用法

### 1.1.3. References

- 官网: https://go-kratos.dev/
- Github: https://github.com/go-kratos/kratos

## 1.2. Go zero

### 1.2.1. 简介

zerolog 是一个高性能、零内存分配的 Go 日志库，结构化日志记录，即日志输出打印的格式为 JSON。

### 1.2.2. 用法

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



### 1.2.3. References

- Go packages: https://pkg.go.dev/github.com/rs/zerolog
- Github: https://github.com/rs/zerolog
- Better Stack 教程：https://betterstack.com/community/guides/logging/zerolog/ 



文档页面排版很好看，图标配色好看，很值得自己学习！！！

https://learning-cloud-native-go.github.io/

# 2. 工程

## 2.1. wire

### 2.1.1. 简介

[wire](https://github.com/google/wire) 是 google 开源用 Go 语言写的用于编译时依赖注入的（dependency injection）代码生成工具。它能够根据你的代码，生成相应的依赖注入 go 代码。

注：依赖注入的工具还有用反射实现的。

### 2.1.2. 用法

Wire通过读取Go代码中的注释和类型信息，生成依赖注入相关的代码。这些生成的代码包括初始化函数、依赖注入容器和相应的依赖关系。生成的代码可以帮助开发人员更容易地管理和解决复杂的依赖关系。

- `wire.NewSet()`: 将多个 provides 放到一个 set 集合中。
- `Injector`:  注入器（Injector）是通过编写一个函数声明来声明的，声明的函数体内是对 `wire.build` 的调用。



### 2.1.3. Reference

- Go packages: https://pkg.go.dev/github.com/google/wire 
- Github: https://github.com/google/wire
- Compile-time Dependency Injection With Go Cloud's Wire: https://go.dev/blog/wire
- Go工程化 - 依赖注入: https://go-kratos.dev/blog/go-project-wire/
- Compile-time Dependency Injection With Go Cloud's Wire: https://go.dev/blog/wire





# 3. 工具

## 3.1. Cobra

### 3.1.1. 简介

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

### 3.1.2. 用法

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



### 3.1.3. References

- Go 官网包：https://pkg.go.dev/github.com/spf13/cobra
- 官网：https://cobra.dev/

## 3.2. viper

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



### 3.2.1. 为什么选择Viper?

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

### 3.2.2. References

- Github：https://github.com/spf13/viper
- Go语言配置管理神器——Viper中文教程：https://www.liwenzhou.com/posts/Go/viper/



## 3.3. Go-toml

### 3.3.1. 简介

Go-toml 是一个操作 TOML 格式的 Go library。

### 3.3.2. References

Github: https://github.com/pelletier/go-toml

Go packages: https://pkg.go.dev/github.com/pelletier/go-toml



## 3.4. GORM

### 3.4.1. 简介

GORM是Golang目前比较热门的数据库ORM操作库，对开发者也比较友好，使用非常方便简单，使用上主要就是把struct类型和数据库表记录进行映射，操作数据库的时候不需要直接手写Sql代码，这里主要介绍**MySQL**数据库。

### 3.4.2. References

- 官网: https://gorm.io/
- Github: https://github.com/go-gorm/gorm
- Go packages: https://pkg.go.dev/gorm.io/gorm

## 3.5. sarama



### 3.5.1. References

- Github: https://github.com/Shopify/sarama



## 3.6. Kafka-go

和sarama一样，segmentio/kafka-go也是一个纯go实现的kafka client，并且在很多公司的生产环境经历过考验，segmentio/kafka-go提供低级conn api和高级api(reader和writer)，以writer为例，相对低级api，它是并发safe的，还提供连接保持和重试，无需开发者自己实现，另外writer还支持sync和async写、带context.Context的超时写等。

不过Writer的sync模式写十分慢，1秒钟才几十条，但async模式就飞快了！

不过和confluent-kafka-go一样，segmentio/kafka-go 也没有像 sarama 那样提供 mock 测试包，我们需要自己建立环境测试。kafka-go 官方的建议时：**在本地启动一个kafka服务，然后运行测试**。在轻量级容器十分流行的时代，**是否需要 mock 还真是一件值得思考的事情**。



### 3.6.1. 对比

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



### 3.6.2. Reference

- Github: https://github.com/segmentio/kafka-go
- Go packages: https://pkg.go.dev/github.com/segmentio/kafka-go
- Go社区主流Kafka客户端简要对比: https://tonybai.com/2022/03/28/the-comparison-of-the-go-community-leading-kakfa-clients/







# 4. errgroup

用于处理 goroutine 中的错误。

### 4.0.1. 用法

Group 的核心能力就在于能够并发执行多个子任务，从调用者的角度，我们只需要传入要执行的函数，签名为：`func() error`即可，非常通用。如果任务执行成功，就返回 nil，否则就返回 error，并且会 cancel 那个新的 Context。



### 4.0.2. References

- Go packages: https://pkg.go.dev/golang.org/x/sync/errgroup



## 4.1. fiber

### 4.1.1. 简介

**Fiber**，一个受[Express](https://github.com/expressjs/express)启发的Golang **Web框架**，建立在[Fasthttp](https://github.com/valyala/fasthttp) 的基础之上。旨在**简化**、**零内存分配**和**高性能**，以及**快速**开发。Go 速度快，占用的内存少，而且性能高，这意味着它也使得 Fiber 框架更快。

### 4.1.2. 用法

#### 4.1.2.1. 🧬 内置中间件

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



#### 4.1.2.2. 🧬 外部中间件

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



### 4.1.3. References

- 官网: https://docs.gofiber.io/
- Go packages: https://pkg.go.dev/github.com/gofiber/fiber/v2
- Github: https://github.com/gofiber/fiber

## 4.2. go-migrate

用 Go 语言编写的数据库迁移命令行工具。

### 4.2.1. 为什么要使用数据库迁移工具

让代码与数据的改变都进行版本控制。



### 4.2.2. Reference

- Github: https://github.com/golang-migrate/migrate



## 4.3. Watermill

### 4.3.1. 简介

[watermill](https://watermill.io/)是 Go 语言的一个异步消息解决方案，它支持消息重传、保存消息，后启动的订阅者也能收到前面发布的消息。`watermill`内置了多种**订阅-发布**实现，包括`Kafka/RabbitMQ`，甚至还支持`HTTP/MySQL binlog`。当然也可以编写自己的订阅-发布实现。此外，它还提供了监控、限流等中间件。



### 4.3.2. References

- 官网：https://watermill.io/
- Github: https://github.com/ThreeDotsLabs/watermill/
- Go packages: https://pkg.go.dev/github.com/ThreeDotsLabs/watermill@v1.2.0/pubsub/gochannel

