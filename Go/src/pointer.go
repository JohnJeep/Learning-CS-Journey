package main

import "fmt"

func main() {
	a := 10
	b := &a

	fmt.Printf("a: %d, type: %T\n", a, a)
	fmt.Printf("b: %p, type: %T\n", b, b)
	fmt.Println("&a:", &a) // a 地址
	fmt.Println("b:", b)   // b 的值
	fmt.Println("&b:", &b) // b 地址
	fmt.Println("*b:", *b) // 取 b 的地址中指向的值
}
