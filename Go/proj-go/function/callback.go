package main

import "fmt"

type FuncType func(int, int) int

// 定义回调函数
// Go 推荐写法：声明时返回值定义一个变量，return时，不用写返回的变量
func Calc(a, b int, call FuncType) (result int) {
	result = call(a, b)
	return
}

// 加法实现
func Add(x, y int) int {
	return x + y
}

// 乘法实现
func Multi(x, y int) int {
	return x * y
}

func main() {
	fmt.Println("Callback...")
	res := Calc(10, 20, Add)
	fmt.Println("Add:", res)

	res = Calc(4, 9, Multi)
	fmt.Println("Multi:", res)
}
