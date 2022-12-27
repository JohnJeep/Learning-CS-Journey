// select 语句类似于 switch 语句，但是select会随机执行一个可运行的case。如果没有case可运行，它将阻塞，直到有case可运行。
// select 是Go中的一个控制结构，类似于用于通信的switch语句。每个case必须是一个通信操作，要么是发送要么是接收。 select 随机执行一个可运行的case。
// 如果没有case可运行，它将阻塞，直到有case可运行。一个默认的子句应该总是可运行的。

package main

import (
	"fmt"
	"time"
)

func main() {
	intChan := make(chan int, 10)
	stringChan := make(chan string, 5)

	for i := 0; i < cap(stringChan); i++ {
		stringChan <- "Hello " + fmt.Sprintf("%d", i) // write
	}

	for i := 0; i < cap(intChan); i++ {
		intChan <- i // write
	}

	for {
		select {
		case valInt := <-intChan:
			fmt.Printf("从int channel read data: %d\n", valInt)
		case valStr := <-stringChan:
			fmt.Printf("从string channel read data: %s\n", valStr)
			time.Sleep(time.Second)
		default:
			fmt.Printf("没有来自管道中的数据可读\n")
			return // 退出循环，不然一值在死循环中
		}
	}
}
