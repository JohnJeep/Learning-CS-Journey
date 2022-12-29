package main

import (
	"log"
)

// 在 main 之前调用，初始化，配置日志参数，用于log包进行正确的输出
func init() {
	log.SetPrefix("TRACE: ") // 日志前缀

	// 标志位控制要写入哪些信息
	log.SetFlags(log.Ldate | log.Lmicroseconds | log.Llongfile)

	// Ldate 日期: 2009/01/23
	// Ltime 时间: 01:23:23
	// Llongfile 显示完整路径的文件名和行号: /a/b/c/d.go:23
	// Lshortfile 只显示最终的文件名和行号: d.go:23
	// LstdFlags = Ldate | Ltime 标准日志记录器的初始值
}

func main() {
	// Println 写到标准日志记录器
	log.Println("Message")

	// Fatalln 在调用 Println()之后会接着调用 os.Exit(1)
	log.Fatalln("Fatal message")

	// Panicln 在调用 Println()之后会接着调用 panic()
	log.Panicln("Panic message")
}
