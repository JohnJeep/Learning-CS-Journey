<!--
 * @Author: JohnJeep
 * @Date: 2025-04-18 15:16:37
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-04-23 17:31:24
 * @Description: Gin framework usage
 * Copyright (c) 2025 by John Jeep, All Rights Reserved. 
-->

# 1. Introduction

Gin 是一个用 Go (Golang) 编写的 Web 框架。 它具有类似 martini 的 API，性能要好得多，多亏了 `httprouter`，速度提高了 40 倍。 如果您需要性能和良好的生产力，您一定会喜欢 Gin。

基于官方 net/http 内建标准库封装的。 

# 2. Fundamental

Gin是一个高性能的HTTP框架，核心可能包括路由、中间件、请求处理、响应处理这些。

## 路由(route)



## 中间件(middleware)

- logger

  Logger 中间件用于记录每个 HTTP 请求的相关信息，如请求方法、请求路径、响应状态码、响应时间等。示例代码如下：

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

- recovery

  Recovery 中间件用于**捕获并恢复处理过程中发生的 panic**，避免因某个请求处理时的 panic 导致整个服务崩溃。在上面使用`gin.Default()`创建引擎时，已经默认使用了该中间件。

- gzip

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

- 自定义中间件

  可以根据自己的需求创建自定义中间件。例如，创建一个简单的身份验证中间件：

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

## request



## response



## 2.1. 用法

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

## 2.2. set mode

```go
var mode string = gin.DebugMode

func init() {
	switch mode {
	case gin.DebugMode:
	case gin.ReleaseMode:
	case gin.TestMode:
	default:
		mode = gin.DebugMode
	}

	gin.SetMode(mode)
}
```


# 3. References

- Github: https://github.com/gin-gonic/gin
- official web: https://gin-gonic.com/
- Go packages: https://pkg.go.dev/github.com/gin-gonic/gin
- official example: https://github.com/gin-gonic/examples
- gin 框架源码解析：https://www.liwenzhou.com/posts/Go/gin-sourcecode/


