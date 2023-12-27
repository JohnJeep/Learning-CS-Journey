package main

import (
	"fmt"
	"runtime"
	"sync"
)

// Wait for the program to finish.
var wg sync.WaitGroup

func main() {
	// acllocate 1 logical processor for the scheduler to use
	// 显示的结果没有交叉
	runtime.GOMAXPROCS(1)

	// NumCPU() 返回当前系统的逻辑处理器数量
	// 给每个可用的核心分配一个逻辑处理器
	// 处理是并行执行的，显示的结果有交叉
	// runtime.GOMAXPROCS(runtime.NumCPU())

	fmt.Println("Main Create Goroutines")
	// 计数器加 2，表示要等待两个 goroutine
	wg.Add(2)

	go showPrime("A")
	go showPrime("B")

	fmt.Println("Main Waiting To Finish")
	// 等待 goroutine 结束
	wg.Wait()

	fmt.Println("\nTerminating Program")
}

func showPrime(prefix string) {
	// 在函数退出时调用 Done 来通知 main 函数工作已经完成
	defer wg.Done()
	for i := 0; i < 100; i++ {
		fmt.Println("prefix:", prefix, "i:", i)
	}
}
