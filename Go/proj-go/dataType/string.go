package main

import (
	"fmt"
	"strings"
)

func main() {
	// UTF-8 中一个ASCII码占一个字节，汉字占三个字节
	s := "你好"
	fmt.Println("s len:", len(s))
	fmt.Println(strings.Contains("contain", "foo"))
	fmt.Println(strings.Contains("contain", "tain"))

	var bytes = []byte("hello")
	fmt.Printf("bytes: %v\n", bytes) // 显示的是字符的 ASCII 码

	// str := string([]byte{97, 98, 99})
	str := string([]byte{97, 98, 99})
	fmt.Printf("byte convert string: %v\n", str)

	comp := strings.EqualFold("xyz", "XYZ") // 不区分大小写
	fmt.Println("two string compare:", comp)
}
