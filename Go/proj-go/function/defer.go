package main

import "fmt"

func fault(x int) int {
	result := 1000 / x
	return result
}

func TestNoDefer() {
	fmt.Println("1111111111111")
	fmt.Println("2222222222222")
	// fault(0) // 出错后程序终止
	fmt.Println("3333333333333")
}

// 带有 defer 关键字的语句，语句执行开始时，会把数据压入 defer 栈中
// 整个函数执行完成后，再从 defer 栈中取出之前压入的数据，按 先进后出的顺序执行
func TestUseDefer() {
	fmt.Println("Start carry defer ...")
	defer fmt.Println("1111111111111")
	defer fmt.Println("2222222222222")
	defer fault(0) // 加 defer后，出错后面有语句仍然可以执行
	// fault(0)
	defer fmt.Println("3333333333333")
}

// 匿名函数与 defer 的使用
func AnonymousDefer() {
	id := 007
	name := "spader"

	defer func() {
		fmt.Printf("匿名函数内部, id = %d, name = %s\n", id, name)
	}()

	fmt.Printf("匿名函数外部, id = %d, name = %s\n", id, name)
}

// 带参数的匿名函数结合 defer
func AnonymousDeferWithParam() {
	id := 007
	name := "spader"

	defer func(a int, b string) {
		fmt.Printf("带参数匿名函数内部, id = %d, name = %s\n", a, b)
	}(id, name) // 调用匿名函数之前，id=007, name="spader" 的值已被传递进来，还没有被执行，等调用defer时才被执行

	// 先执行下面的语句，再执行defer后的匿名函数
	id = 100
	name = "Jole"
	fmt.Printf("带参数匿名函数外部, id = %d, name = %s\n", id, name)
}

func main() {

	TestNoDefer()
	// TestUseDefer()
	AnonymousDefer()
	AnonymousDeferWithParam()
}
