// channel 中带有缓冲，用作计数信号量

package main

import (
	"fmt"
	"sync"
	"time"
)

// 同一时间最多允许 3 个gorouitine 处于活动状态
func main() {
	active := make(chan struct{}, 3) // 作为计数信号量，处于活动状态的最大数为 3
	jobs := make(chan int, 10)       // 作为任务队列，最大容量为 10

	// 生成 8 个任务
	go func() {
		for i := 0; i < 8; i++ {
			jobs <- (i + 1)
		}
		close(jobs) // 关闭 jobs channel
	}()

	// WaitGroup 是一个计数信号量，可以用来记录并维护运行的 goroutine
	// 如果 WaitGroup 的值大于 0，Wait 方法就会阻塞。
	var wg sync.WaitGroup // 用于等待所有 goroutine 完成

	// 从 jobs channel 中读取任务
	for j := range jobs {
		wg.Add(1) // 增加等待 goroutine 的数量
		go func(j int) {
			defer wg.Done() // 函数退出时调用 Done 来通知 main 函数工作已经完成
			active <- struct{}{}
			fmt.Println("handle job", j)
			time.Sleep(2 * time.Second)
			<-active
			wg.Done()
		}(j)
	}
	wg.Wait() // 等待 goroutine 结束
}
