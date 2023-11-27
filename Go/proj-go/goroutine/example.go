// 1. 开启一个 writeData 协程，向管道 intChan 中写入50个整数
// 2. 开启一个 readData 协程，从 intChan 管道中读 writeData 协程中写入的数据
// 3. WriteData 协程与 readData 协程操作是同一个管道
// 4. 主线程需要等 writeData 协程和 readData 协程都完成后才能退出
// 现象：读写管道交叉进行

package main

import (
	"fmt"
)

func WriteData(intChan chan<- int) {
	for i := 0; i < cap(intChan); i++ {
		intChan <- i
		fmt.Println("Write data:", i)
	}
	close(intChan) // 写完后直接关闭管道，避免for循环读时发生 deadlock
}

func ReadData(intChan <-chan int, exitChan chan bool) {
	for {
		//对于已关闭的管道和空管道执行，value, ok := <-intChan 判断管道，关闭和为空，则为 false
		if value, ok := <-intChan; !ok {
			break // 读失败直接退出
		} else {
			fmt.Println("Read channel data:", value)
		}
	}

	// 数据从管道中读完后，往管道中写一个标志位
	exitChan <- true
	close(exitChan)
}

func main() {
	c1 := make(chan int, 50)
	exitChan := make(chan bool, 1)

	go WriteData(c1)
	go ReadData(c1, exitChan)

	// 主线程中一直在扫描退出管道中的值是否发生了变化
	// 值为 true 说明，两个协程完成了工作，主线程可以结束退出
	for {
		if _, ok := <-exitChan; !ok {
			break
		}
	}
}
