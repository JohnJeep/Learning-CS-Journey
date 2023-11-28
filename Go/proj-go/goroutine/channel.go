package main

import (
	"fmt"
	"math/rand"
	"time"
)

func longRequest() <-chan int32 {
	r := make(chan int32)
	go func() {
		time.Sleep(time.Second * 3) // 模拟一个工作负载负载
		r <- rand.Int31n(100)
	}()

	return r
}

func sumSquares(a, b int32) int32 {
	return a*b + b*b
}

func version1() {
	rand.Seed(time.Now().UnixNano())
	a, b := longRequest(), longRequest()
	fmt.Println(sumSquares(<-a, <-b)) // 每个通道读取操作将阻塞到请求返回结果为止，两个实参总共需要大约3秒钟
}

func longRequest2(r chan<- int32) {
	time.Sleep(time.Second * 3)
	r <- rand.Int31n(100)
}

func version2() {
	rand.Seed(time.Now().UnixNano())
	ra, rb := make(chan int32), make(chan int32)
	go longRequest2(ra)
	go longRequest2(rb)

	fmt.Println(sumSquares(<-ra, <-rb))
}

func main() {
	version2()
}
