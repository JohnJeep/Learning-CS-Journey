// 获取可执行程序执行时的参数个数

package main

import (
	"fmt"
	"os"
)

func main() {
	list := os.Args // 参数列表，类型为切片

	for i, v := range list {
		fmt.Printf("index = %d, args = %s\n", i, v)
	}

}
