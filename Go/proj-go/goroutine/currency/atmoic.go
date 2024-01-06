// 原子函数能够以很底层的加锁机制来同步访问整型变量和指针

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
		// val := counter // 存在竞争问题
		atmoic.AddInt64(&counter, 1) // 原子操作，安全的加 1 操作

		// 当前 goroutine 从线程退出，并放回到队列，给其他 goroutine 运行的机会
		runtime.Gosched()
	}
}
