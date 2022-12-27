package main

import (
	"fmt"
)

func TestArray() {
	type Currency int
	const (
		USD Currency = iota // 美元
		EUR                 // 欧元
		GBP                 // 英镑
		RMB                 // 人民币
	)
	symbol := [...]string{USD: "$", EUR: "€", GBP: "￡", RMB: "￥"}
	fmt.Println(RMB, symbol[RMB]) // "3 ￥"
}

func main() {
	var arr0 [5]int // 定义数组

	for index, value := range arr0 {
		fmt.Printf("index=%d, value=%d\n", index, value)

	}

	var arr1 = [...]int{1, 2, 3, 4, 5} // 长度为...，数据的长度由初始值中的个数来确定的
	var arr2 = [5]int{2: 11, 4: 33}    // 使用索引号初始化元素

	// 直接打印数组
	fmt.Println("arr1:", arr1)
	fmt.Println("arr2:", arr2)

	// 遍历数组，忽略索引
	for _, v := range arr1 {
		fmt.Println("arr1:", v)
	}

	for i2, v2 := range arr2 {
		fmt.Println("arr2:", i2, v2)
	}

	TestArray()

}
