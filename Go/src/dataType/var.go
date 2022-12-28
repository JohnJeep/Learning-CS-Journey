package main

import "fmt"

// 变量声明
// var sum int   // 默认初始值为 0
var sum = 0 // 等价于上面的表达式，声明时省略了变量的 类型

func add() {
	// 短变量声明 x y，只在函数内有效
	x := 10
	y := 20
	sum = x + y

	fmt.Println("add sum:", sum)
}

// 带有返回值的函数，返回值为：(int, string)
func foo() (int, string) {
	return 10, "Hello"
}

func anonymous() {
	a, _ := foo() // 匿名变量声明 _ 用于占位，表示忽略值，不占用命名空间，不分配内存
	_, b := foo()

	fmt.Println("a =", a)
	fmt.Println("b =", b)
}

func main() {
	fmt.Println("globle sum:", sum)

	add()
	anonymous()
}
