// 用 Gin 框架创建 HTTP Server，client 用哪种方法发送的数据
// server 接收到响应后，将接收到的方法、URL以及请求的包体都
// 返回给 client，达到 client 发送什么，server 就回什么的效果

package main

import (
	"github.com/gin-gonic/gin"
)

func main() {
	// 创建Gin路由引擎
	r := gin.Default()

	// 使用Gin的Any方法，接受所有HTTP方法（GET、POST、PUT、DELETE等）的请求
	r.Any("/*action", func(c *gin.Context) {
		// 解析客户端发送的JSON请求数据
		requestURL := c.Request.URL.Path
		requestMethod := c.Request.Method
		httpVersion := c.Request.Proto

		// 构建响应
		requestData := map[string]interface{}{
			"message":     "Hello from Gin!",
			"request_url": requestURL,
			"method":      requestMethod,
			"version":     httpVersion,
		}

		if err := c.ShouldBindJSON(&requestData); err != nil {
			c.JSON(400, gin.H{"error": "Invalid JSON data"})
			return
		}

		// 返回相同的JSON数据作为响应
		c.JSON(200, requestData)
	})

	// 启动HTTP服务器，监听端口
	r.Run(":8089")
}
