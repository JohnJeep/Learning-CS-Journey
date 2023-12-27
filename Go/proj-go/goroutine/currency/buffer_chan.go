// buffered channel
//
// 有缓冲的通道（buffered channel）是一种在被接收前能存储一个或者多个值的通道。
// 这种类型的通道并不强制要求 goroutine 之间必须同时完成发送和接收，通道会阻塞发送和接收动作的条件也会不同。
// 只有在通道中没有要接收的值时，接收动作才会阻塞。只有在通道没有可用缓冲区容纳被发送的值时，发送动作才会阻塞。
//
// 这导致有缓冲的通道和无缓冲的通道之间的一个很大的不同：
// 无缓冲的 channel 保证进行发送和接收的 goroutine 会在同一时间进行数据交换；有缓冲的 channel 没有这种保证。
//
// 有缓冲的 channel 可用作异步操作

package main

import (
	"fmt"
	"math/rand"
	"sync"
	"time"
)

const (
	numberGoroutines = 4  // 要使用的 goroutine 的数量
	taskLoad         = 10 // 要处理的工作的数量
)

var wg sync.WaitGroup

func init() {
	source := rand.NewSource(time.Now().UnixNano())
	rand.New(source)
}

func bufferChan() {
	buffered := make(chan string, 10) // 有缓冲通道，缓冲区大小为 10

	// send a string through the channel
	buffered <- "Gopher"

	// receive a string from the channel
	value := <-buffered
	fmt.Println("value:", value)
}

func main() {
	tasks := make(chan string, taskLoad)

	// 启动 goroutine 来处理工作
	wg.Add(numberGoroutines)
	for gr := 1; gr <= numberGoroutines; gr++ {
		go worker(tasks, gr)
	}

	// 增加一组要完成的工作
	for post := 1; post <= taskLoad; post++ {
		tasks <- fmt.Sprintf("Task: %d", post)
	}

	// 当所有工作都处理完时关闭通道，以便所有 goroutine 退出
	// 1. 通道必须在使用完毕后关闭，否则会引发 panic
	// 2. 通道关闭后，仍然可以从通道 receive 数据，但不能再向通道 send 数据
	// 3. 能够从已经关闭的通道接收数据这一点非常重要，因为这允许通道关闭后依旧能取出其中缓冲的全部值，而不会有数据丢失。
	// 4. 从一个已经关闭且没有数据的通道 里获取数据，总会立刻返回，并返回一个通道类型的零值
	// 5. 如果在获取通道时还加入了可选的标志，就能得到通道的状态信息
	close(tasks)

	// 等待所有工作完成
	wg.Wait()
}

// 处理过程
func worker(tasks chan string, worker int) {
	defer wg.Done() // 通知 main 函数工作已经完成

	for {
		// 等待分配工作
		task, ok := <-tasks
		if !ok {
			// 这意味着通道已经空了，并且已被关闭
			fmt.Printf("Worker: %d: Shutting Down\n", worker)
			return
		}

		// 显示开始工作
		fmt.Printf("Worker: %d: Started %s\n", worker, task)

		// 随机等一段时间来模拟工作
		sleep := rand.Int63n(100)
		time.Sleep(time.Duration(sleep) * time.Millisecond)

		// 显示工作已完成
		fmt.Printf("Worker: %d: Completed %s\n", worker, task)
	}
}
