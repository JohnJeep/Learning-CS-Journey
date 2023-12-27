// 互斥锁来解决共享资源的同步访问
// 互斥锁用于在代码上创建一个临界区，保证同一时间只有一个 goroutine 可以 执行这个临界区代码。

package main

import (
	"fmt"
	"runtime"
	"sync"
)

var (
	counter int            // 是一个共享资源
	wg      sync.WaitGroup // 等待程序结束
	mutex   sync.Mutex     // 互斥锁
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
		// 同一时刻只允许一个 goroutine 进入这个临界区
		mutex.Lock() // 加锁
		{
			val := counter

			// 当前 goroutine 从线程退出，并放回到队列，给其他 goroutine 运行的机会
			runtime.Gosched()

			val++
			counter = val // 写入 counter
		}
		mutex.Unlock() // 解锁
	}
}
