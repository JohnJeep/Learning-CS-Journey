// 程序产生异常，结合 recover()、defer 进行异常处理 Panic

package main

import "fmt"

func TestRecover(index int) {
	defer func() {
		// 出错打印 Panic 异常的错误，使程序从异常中恢复运行
		if err := recover(); err != nil {
			fmt.Println("Panic error:", err)
		} else {
			fmt.Println("index value:", index)
		}
	}()

	var arr [10]int
	arr[index] = 100
}

func main() {

	TestRecover(11)
	TestRecover(8)
}
