// 多个协程，一个协程发生 Panic

package main

import (
	"fmt"
	"time"
)

func Working() {
	for i := 0; i < 10; i++ {
		fmt.Printf("I'm working of %d\n", i)
		time.Sleep(time.Second)
	}
}

func Doing() {
	var myMap map[int]string

	// 对发生的异常做处理
	defer func() {
		if err := recover(); err != nil {
			fmt.Println("Catch goroutine Doing error...")
		}
	}()

	myMap[0] = "Goood" // 发生错误，使用之前未用 make 创建
	fmt.Println("myMap", myMap)
}

func main() {
	go Working()
	go Doing()

	// 在主函数结束之前，让其它的协程能工作
	for i := 0; i < 10; i++ {
		fmt.Println("Main:", i)
		time.Sleep(time.Second)
	}
}
