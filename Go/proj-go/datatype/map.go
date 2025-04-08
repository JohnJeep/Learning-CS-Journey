package main

import "fmt"

func main() {
	// 定义一个变量，类型为 map[int]string
	var m1 map[int]string
	fmt.Printf("m1: %v, len: %d\n", m1, len(m1))

	// make 创建map
	m2 := make(map[int]string)
	fmt.Printf("m2: %v, len: %d\n", m2, len(m2))

	// make 创建map，并指定长度
	m3 := make(map[int]string, 10) // 只指定了map的容量，但map实际的len由map中的数据决定的
	fmt.Printf("m3: %v, len: %d\n", m3, len(m3))

	// map 赋值
	m2[10] = "ID"
	m2[20] = "name"
	fmt.Printf("m2: %v, len: %d\n", m2, len(m2))

	// map 遍历
	for key, val := range m2 {
		fmt.Printf("key: %d, value: %s\n", key, val)
	}

	// map 中元素删除
	m2[15] = "go"
	m2[18] = "salary"
	fmt.Printf("m2: %v, len: %d\n", m2, len(m2))

	fmt.Printf("m2 subscript access: %s\n", m2[15]) // 下标访问

	delete(m2, 15)
	fmt.Printf("delete m2: %v, len: %d\n", m2, len(m2))
}
