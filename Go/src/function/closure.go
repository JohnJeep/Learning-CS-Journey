// 匿名函数以引用的方式捕获外部变量，这种形式叫做闭包
// 闭包特点：
// 1. 闭包不关心捕获的变量或常量是否超出了作用域，只要闭包还在使用变量，这些变量就一直存在
// 2、变量的生命周期不由它的作用域决定

package main

import "fmt"

// 函数的返回值为匿名函数
func closure() func() int {
	var x int

	return func() int {
		x++
		return x * x
	}
}

// 普通函数
func Normal() int {
	var x int
	x++
	return x * x
}

func TestNormal() {
	f := Normal()
	fmt.Println("Normal, x:", f)
	fmt.Println("Normal, x:", f)
	fmt.Println("Normal, x:", f)
	fmt.Println("Normal, x:", f)
	fmt.Println("Normal, x:", f)
	fmt.Println("Normal, x:", f)
}

func TestClosure() {
	f := closure()
	fmt.Println("Closure, x:", f())
	fmt.Println("Closure, x:", f()) // 第一次调用完后，x 值还存在，没有被清0
	fmt.Println("Closure, x:", f())
	fmt.Println("Closure, x:", f())
	fmt.Println("Closure, x:", f())
	fmt.Println("Closure, x:", f())
}

func main() {
	TestClosure()
	TestNormal()
}
