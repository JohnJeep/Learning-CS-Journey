package main

import (
	"fmt"
	"time"
)

func main() {
	for {
		timeout := time.After(2 * time.Second) // 创建一个 2 秒的定时器
		select {
		case <-timeout:
			fmt.Println("deal timer")
		}
	}
}
