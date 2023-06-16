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

zerolog 是一个高性能、零内存分配的 Go 日志库，日志输出打印的格式为 JSON。

### 1.2.2. 用法

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

### 2.1.3. Reference

- Go packages: https://pkg.go.dev/github.com/google/wire 
- Github: https://github.com/google/wire
- Compile-time Dependency Injection With Go Cloud's Wire: https://go.dev/blog/wire
- Go工程化 - 依赖注入: https://go-kratos.dev/blog/go-project-wire/





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

## 4.2. 🧬 内置中间件

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



## 4.3. 🧬 外部中间件

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



### 4.3.1. References

- 官网: https://docs.gofiber.io/
- Go packages: https://pkg.go.dev/github.com/gofiber/fiber/v2
- Github: https://github.com/gofiber/fiber

