// 模拟条件竞争
// Golang 中用竞争检测器标志来编译程序，可以检测出是否存在竞争条件
// go build -race xxx.go
//
// 一种修正代码、消除竞争状态的办法是，使用 Go 语言提供的锁机制，
// 来锁住共享资源，从 而保证 goroutine 的同步状态

// 采用的方法
// 1. 互斥锁
// 2. 原子函数
// 3. 互斥锁
// 4. channel

package main

import (
	"fmt"
	"runtime"
	"sync"
)

var (
	counter int // 是一个共享资源
	wg      sync.WaitGroup
)

// 变量counter会进行4次读和写操作，每个 goroutine 执行两次
// 但是，程序终止时，counter 变量的值为 2。
func main() {
	wg.Add(2)

	go incCounter(1)
	go incCounter(2)

	wg.Wait() // 等待 goroutine 结束
	fmt.Println("Final Counter:", counter)
}

func incCounter(id int) {
	defer wg.Done() // 在函数退出时调用 Done 来通知 main 函数工作已经完成

	for i := 0; i < 2; i++ {
		val := counter

		// 当前 goroutine 从线程退出，并放回到队列，给其他 goroutine 运行的机会
		runtime.Gosched()

		val++
		counter = val // 写入 counter
	}
}
