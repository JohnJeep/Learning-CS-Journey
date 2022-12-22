package main

import "fmt"

// 多返回值
func MultiReturn(x int, y int) (multi, avg int) {
	multi = x * y
	avg = (x + y) / 2

	return multi, avg
}

func TestMultiReturn() {
	multi, avg := MultiReturn(10, 6)
	fmt.Println("Multi return value:", multi, avg)
}

// 可变参数：函任意个数接收的参数个数为
func Sum(val ...int) int {
	total := 0
	for _, v := range val {
		total += v
	}
	return total
}

func TestVarParam() {

	fmt.Println("Zero parameter:", Sum())
	fmt.Println("One parameter:", Sum(10))
	fmt.Println("Three parameter:", Sum(1, 3, 5, 7))

	// 传递参数为切片
	values := []int{2, 4, 6, 8}
	fmt.Println("Param of slice:", Sum(values...))
}

func main() {
	TestVarParam()
	TestMultiReturn()
}
