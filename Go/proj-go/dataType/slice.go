package main

import "fmt"

// 在原内存空间将[]int类型的slice反转， 而且它可以用于任意长 度的slice
func SliceReverse(s []int) {
	for i, j := 0, len(s)-1; i < j; i, j = i+1, j-1 {
		s[i], s[j] = s[j], s[i]
	}
}

func TestAppend() {
	var r []rune
	for _, val := range "Hello, word" {
		r = append(r, val)
	}
	fmt.Printf("%q\n", r)
}

func main() {
	// monthice 是一个可变长的 array，是 array 的一个引用
	// var s []string // 声明一个 slice ，未初始化

	// 声明数组
	month := [...]string{
		1: "January", 2: "February", 3: "March", 4: "April", 5: "May", 6: "June",
		7: "July", 8: "August", 9: "September", 10: "October", 11: "November", 12: "December",
	}

	for i := 0; i < len(month); i++ {
		fmt.Println(month[i]) // 第0个元素自动被设置为空字符串
	}

	for index, v := range month {
		fmt.Println(index, v)
	}

	Q2 := month[4:7]
	summer := month[6:9]
	fmt.Println("Q2:", Q2)
	fmt.Println("summer:", summer) // summer: [June July August]

	// fmt.Println("Out of range:", summer[:15]) // 产生一个 panic 异常

	// summer 长度不够时，新的 slice 长度变大
	extendSummer := summer[:5]
	fmt.Println("extend slice:", extendSummer) // extend slice: [June July August September October]

	// 反转数组
	a := [...]int{1, 2, 3, 4, 5}
	SliceReverse(a[:])
	fmt.Println("reverse slice:", a)

	TestAppend()

	// 容器字面量初始化
	s := []string{"one", "two", "there", "four"}
	fmt.Println(s)
}
