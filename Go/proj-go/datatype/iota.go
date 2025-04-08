package main

import "fmt"

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

func main() {
	PrintConstIota()
	PrintConstIotaShift()
	PrintConstIotaMulti()
}
