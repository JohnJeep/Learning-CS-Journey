package main

import "fmt"

type Point struct {
	x int
	y int
}

// 断言示例
func TestAssert() {
	var a interface{}
	var p1 Point = Point{10, 20}
	a = p1 // ok
	var p2 Point
	// p2 = a // error
	p2 = a.(Point) // 使用类型断言
	fmt.Printf("p2 type: %T, p2: %v\n", p2, p2)
}

// 断言中出错加判断机制
func TestIfAssert() {
	// var f1 float32 = 9.9
	var f1 float64 = 9.9
	var t1 interface{}

	t1 = f1 // 基础类型转化给空接口，ok
	// var f2 float64
	// f2 = t1.(float64) // 执行此句发生 Painc
	if f2, ok := t1.(float64); ok {
		fmt.Printf("f2 type: %T, f2: %v\n", f2, f2)
	} else {
		fmt.Println("Panic error")
	}

	fmt.Println("执行后续内容...")
}

func TypeJudge(iterms ...interface{}) {
	for i, v := range iterms {
		switch v.(type) { // 使用断言判断
		case float32:
			fmt.Printf("第 %d 个参数，类型 %T\n", i+1, v)
		case float64:
			fmt.Printf("第 %d 个参数，类型 %T\n", i+1, v)
		case int:
			fmt.Printf("第 %d 个参数，类型 %T\n", i+1, v)
		case string:
			fmt.Printf("第 %d 个参数，类型 %T\n", i+1, v)
		case bool:
			fmt.Printf("第 %d 个参数，类型 %T\n", i+1, v)
		default:
			fmt.Printf("无效的类型\n")
		}
	}
}

// 判断传入的参数类型
func TestParamType() {
	var f1 float32 = 10.2
	var f2 float64 = 66.5
	f3 := 100
	f4 := "mystring"
	f5 := false

	TypeJudge(f1, f2, f3, f4, f5)
}

func main() {
	TestAssert()
	TestIfAssert()
	TestParamType()
}
