package main

import "fmt"

// 模拟程序发生错误
func TestPanic() {
	var s []string

	s[0] = "hello" // error, 调用切片之前没有用 make 去创建，发生Panic

	fmt.Println("slice:", s)
}

func main() {
	TestPanic()
}
