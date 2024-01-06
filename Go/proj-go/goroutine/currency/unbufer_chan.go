// 原子函数和互斥锁都能工作，但是依靠它们都不会让编写并发程序变得更简单，更不容易出错，或者更有趣
// 使用 channel，通过发送和接收需要共享的资源，在 goroutine 之间做同步
//
// 当一个资源需要在 goroutine 之间共享时，通道在 goroutine 之间架起了一个管道，并提供了
// 确保同步交换数据的机制。声明通道时，需要指定将要被共享的数据的类型。可以通过通道共享
// 内置类型、命名类型、结构类型和引用类型的值或者指针

// unbuffered channel：是指在接收前没有能力保存任何值的通道。这种类型的通
// 道要求发送 goroutine 和接收 goroutine 同时准备好，才能完成发送和接收操作。如果两个 goroutine
// 没有同时准备好，通道会导致先执行发送或接收操作的 goroutine 阻塞等待。这种对通道进行发送
// 和接收的交互行为本身就是同步的。其中任意一个操作都无法离开另一个操作单独存在。

package main

import (
	"fmt"
	"sync"
	"time"
)

func unbufferChan() {
	unbuffered := make(chan int) // 无缓冲通道
	unbuffered <- 11
	// valInt := <-unbuffered
	// fmt.Println("unbuffered:", valInt)
	fmt.Println("unbuffered:", unbuffered)
}

// 等待程序结束
var wg sync.WaitGroup

// 模拟接力赛跑
func RelayRace() {
	// 创建一个无缓冲通道
	baton := make(chan int)

	wg.Add(1)        // 为最后一位跑步者将计数加 1
	go Runner(baton) // 第一位跑步者持有接力棒

	// 将接力棒交给跑步者，开始比赛
	baton <- 1

	// 等待比赛结束，最后一位跑步者跑完即结束
	wg.Wait()
}

// 模拟接力比赛中的一位跑步者
func Runner(baton chan int) {
	var newRunner int

	// 等待接力棒
	runner := <-baton

	// 开始绕着跑道跑步
	fmt.Printf("Runner %d Running With Baton\n", runner)

	// 创建下一位跑步者
	if runner != 4 {
		newRunner = runner + 1
		fmt.Printf("Runner %d To The Line\n", newRunner)
		go Runner(baton)
	}

	// 围绕跑道跑
	time.Sleep(100 * time.Millisecond)

	// 比赛结束了吗
	if runner == 4 {
		fmt.Printf("Runner %d Finished, Race Over\n", runner)
		wg.Done() // 执行完后，WaitGroup 计数减 1
		return
	}

	// 将接力棒交给下一位跑步者
	fmt.Printf("Runner %d Exchange With Runner %d\n", runner, newRunner)

	// 接力棒会交到下一个已经在等待的跑步者手上
	// 在这个时候，goroutine 会被锁住，直到交接完成
	baton <- newRunner
}

func main() {
	// unbufferChan()
	RelayRace()
}
