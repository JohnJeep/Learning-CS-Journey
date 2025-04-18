/*
 * @Author: JohnJeep
 * @Date: 2025-04-18 17:41:04
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-04-18 17:48:00
 * @Description: Gin framework example
 * Copyright (c) 2025 by John Jeep, All Rights Reserved.
 */

package main

import (
	"net/http"

	"github.com/gin-gonic/gin"
)

func rootHandler(c *gin.Context) {
	c.String(http.StatusOK, "Hello, World!")
}

func aboutHandler(c *gin.Context) {
	c.String(http.StatusOK, "This is athe about Page.")
}

func main() {
	// 创建一个默认的 Gin 路由引擎
	r := gin.Default()

	// 注册根路径的 GET 请求处理函数
	r.GET("/", rootHandler)
	// 注册 /about 路径的 GET 请求处理函数
	r.GET("about", aboutHandler)

	// start the server, listen port 8080
	r.Run(":8080")
}
