package main

import "fmt"

// 常量声明
const PI = 3.14

// 多个常量声明
const (
	// 同时声明多个常量时，如果省略了值则表示和上面一行的值相同
	// n1 n2 n3 的值都为 100
	n1 = 100
	n2
	n3
)

func PrintConstVar() {
	fmt.Println("const n1:", n1)
	fmt.Println("const n2:", n2)
	fmt.Println("const n3:", n3)
}

func main() {
	PrintConstVar()
	fmt.Println("const PI:", PI)
}
