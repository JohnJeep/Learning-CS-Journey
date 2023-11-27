// 创建自定义的日志记录器

package main

import (
	"io"
	"io/ioutil"
	"log"
	"os"
)

// 等级声明了 4 个 Logger 类型的指针变量
var (
	Trace   *log.Logger
	Info    *log.Logger
	Warning *log.Logger
	Error   *log.Logger
)

// 创建每个Logger类型的值并将其地址赋给每个变量
func init() {
	file, err := os.OpenFile("errors.txt", os.O_CREATE|os.O_RDWR|os.O_APPEND, 0644)
	if err != nil {
		log.Fatalln("Failed to open errors log file:", err)
	}

	// log.New() 创建并正确初始化一个 Logger 类型的值，返回新创建值的地址
	// Discard 是一个 io.Writer，所有的 Write 调用都不会有动作，但是会成功返回
	// 当某个等级的日志不重要时，使用 Discard 变量可以禁用这个等级的日志。
	Trace = log.New(ioutil.Discard, "T: ", log.Ldate|log.Ltime|log.Lshortfile)
	// Trace = log.New(os.Stdout, "T: ", log.Ldate|log.Ltime|log.Lshortfile)

	// stdout 作为日志输出
	Info = log.New(os.Stdout, "I: ", log.Ldate|log.Ltime|log.Lshortfile)
	Warning = log.New(os.Stdout, "W: ", log.Ldate|log.Ltime|log.Lshortfile)

	// io.MultiWriter(): 返回一个 io.Writer 接口类型值，这个值包含之前打开的文件 file，以及 stderr
	// 同时将错误写到 stderr 和文件中
	Error = log.New(io.MultiWriter(file, os.Stderr), "E: ", log.Ldate|log.Ltime|log.Lshortfile)
}

func main() {
	Trace.Println("I'm trace log")
	Info.Println("I'm info log")
	Warning.Println("I'm warning log")
	Error.Println("I'm error log")
}
