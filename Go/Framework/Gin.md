<!--
 * @Author: JohnJeep
 * @Date: 2025-04-18 15:16:37
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-04-18 17:05:44
 * @Description: Gin framework usage
 * Copyright (c) 2025 by John Jeep, All Rights Reserved. 
-->

# 1. Introduction

Gin 是一个用 Go (Golang) 编写的 Web 框架。 它具有类似 martini 的 API，性能要好得多，多亏了 httprouter，速度提高了 40 倍。 如果您需要性能和良好的生产力，您一定会喜欢 Gin。

基于官方 net/http 内建标准库封装的。 


# 2. Fundamental

核心要点

- 路由
- 中间件
- 请求处理
- 响应处理

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


## 2.2. 设置 mode

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


