package main

import (
	"fmt"
	"time"
)

func goroutines() {
	for i := 0; i < 10; i++ {
		fmt.Printf("Goroutine, 第 %d 次, Hello word\n", i+1)
		time.Sleep(time.Second) // 每隔一秒打印
	}
}

func main() {
	go goroutines() // 创建一个协程，与主线程同一时间执行

	for i := 0; i < 10; i++ {
		fmt.Printf("main, 第 %d 次, Hello, go\n", i+1)
		time.Sleep(time.Second)
	}
}
