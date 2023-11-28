package main

import (
	"bytes"
	"fmt"
	"os"
)

func add(x *int) {
	*x += *x
}

func main() {
	// 创建一个 Buffer 值，并将一个字符串写入 Buffer
	// 使用实现 io.Writer 的 Write 方法
	var b bytes.Buffer
	b.Write([]byte("Hello "))

	fmt.Fprintf(&b, "Word!")

	b.WriteTo(os.Stdout)

	num := 10
	add(&num)
	fmt.Println("num", num)
}
