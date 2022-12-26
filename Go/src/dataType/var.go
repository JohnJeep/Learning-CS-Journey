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

// 常量中的iota，go 中的常量计数器
// 位于内部第一行被置为0，每新增一行 iota 引用次数加 1
// 常用于枚举中
const (
	t1 = iota // 0
	t2        // 1
	_         // 跳过 2
	t4        // 3
	t5 = iota // 中间插入 iota，4
	t6        // 5
)

func PrintConstIota() {
	fmt.Println(t1, t2, t4, t5, t6)
}

// 定义数量级
const (
	_  = iota
	KB = 1 << (10 * iota)
	MB = 1 << (10 * iota)
	GB = 1 << (10 * iota)
	TB = 1 << (10 * iota)
	PB = 1 << (10 * iota)
)

func PrintConstIotaShift() {
	fmt.Println(KB, MB, GB, TB, PB)
}

// 多个iot定义在一行
const (
	a, b = iota + 1, iota + 2 // a=0+1, b=0+2
	c, d                      // c=1+1, c=1+2
	e, f                      // e=2+1, f=2+2
)

func PrintConstIotaMulti() {
	fmt.Println(a, b, c, d, e, f)
}

// 变量声明
// var sum int   // 默认初始值为 0
var sum = 0 // 等价于上面的表达式，声明时省略了变量的 类型

func add() {
	// 短变量声明 x y，只在函数内有效
	x := 10
	y := 20
	sum = x + y

	fmt.Println("sum:", sum)
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
	fmt.Println("Start ...")

	add()
	anonymous()
	PrintConstVar()
	PrintConstIota()
	PrintConstIotaShift()
	PrintConstIotaMulti()
}
