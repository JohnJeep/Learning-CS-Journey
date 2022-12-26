package main

import "fmt"

func TestChannel() {
	//  声明管道
	var intChan chan int

	// 创建管道
	intChan = make(chan int, 4)
	fmt.Printf("channel type: %T, value: %v, address of channel: %v\n", intChan, intChan, &intChan)
	fmt.Printf("Before channel cap: %d, len: %d\n", cap(intChan), len(intChan))

	// 管道中读取数据的原则：先进先出，即队列
	// 管道中写数据
	num := 100
	intChan <- num
	intChan <- 200
	fmt.Printf("After channel cap: %d, len: %d\n", cap(intChan), len(intChan))

	// 从管道中取数据
	data1 := <-intChan
	fmt.Println("Take out data1:", data1)
	data2 := <-intChan
	fmt.Println("Take out data2:", data2)

	// 管道中的数据已取完，报错 deadlock 错误
	// data3 := <-intChan
	// fmt.Println("Take out data3:", data3)
}

func TestChannelCloseIter() {
	// 管道的关闭

	// 管道的遍历
}

func main() {
	TestChannel()
}
