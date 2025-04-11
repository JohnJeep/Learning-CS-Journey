/*
 * @Author: JohnJeep
 * @Date: 2025-04-11 15:04:02
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-04-11 17:29:20
 * @Description: 创建5个协程顺序打印，保证协程不能死锁
 * Copyright (c) 2025 by John Jeep, All Rights Reserved.
 */
package main

import (
	"fmt"
	"sync"
)

func single() {
	ch := make(chan int)

	go func() {
		for i := 0; i < 4; i++ {
			ch <- i
		}
		close(ch)
	}()

	for val := range ch {
		fmt.Println(val)
	}
}

func version01() {
	count := 5
	ch := make([]chan struct{}, count)
	var wg sync.WaitGroup
	done := make(chan struct{})

	for i := 0; i < count; i++ {
		ch[i] = make(chan struct{})
		wg.Add(1)
		go func(id int) {
			defer wg.Done()

			// wait for the previous goroutine to finish
			if id != 0 {
				<-ch[id-1]
			}
			fmt.Printf("Goroutine %d\n", id+1)

			// notify the next goroutine
			if id < count-1 {
				ch[id] <- struct{}{}
			} else {
				// 最后一个协程完成后向 done 通道发送信号
				done <- struct{}{}
			}
		}(i)
	}

	// Start the first goroutine
	if count > 0 {
		ch[0] <- struct{}{}
	}

	// 等待所有协程完成
	go func() {
		wg.Wait()
		close(done)
	}()

	// 等待完成信号
	<-done
}

func version02() {
	var wg sync.WaitGroup
	// 创建一个初始值为 1 的通道
	ch := make(chan int, 1)
	ch <- 1
	// 协程数量
	numGoroutines := 5
	wg.Add(numGoroutines)
	for i := 1; i <= numGoroutines; i++ {
		go func(id int) {
			defer wg.Done()
			for {
				// 从通道接收数据
				current, ok := <-ch
				if !ok {
					return
				}
				if current == id {
					fmt.Printf("协程 %d 正在打印\n", id)
					if id < numGoroutines {
						// 发送下一个协程的编号
						ch <- id + 1
					} else {
						// 关闭通道
						close(ch)
					}
					return
				}
				// 若不是当前协程的编号，将数据放回通道
				ch <- current
			}
		}(i)
	}
	// 等待所有协程完成
	wg.Wait()
}

func main() {
	// single()

	// version01()
	version02()
}
