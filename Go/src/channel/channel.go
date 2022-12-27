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

type Cat struct {
	Name string
	Age  int
}

// 管道中存放多种数据类型
func TestSaveCompositeType() {
	allChan := make(chan interface{}, 3)

	allChan <- 100
	allChan <- Cat{"Tom", 3}
	allChan <- "Beautiful"

	// 取数据
	<-allChan        // 取出管道中的第一个数据，丢弃
	tmp := <-allChan // 从管道中取数据，tmp 是接口类型
	// 直接取是错误的，编译器报错，要用类型断言转化后才可以
	// fmt.Printf("Cat name: %s, age: %d\n", tmp.Name, tmp.Age)

	cat1 := tmp.(Cat) // 类型断言
	fmt.Printf("Cat name: %s, age: %d\n", cat1.Name, cat1.Age)
}

func TestChannelCloseIter() {
	c1 := make(chan int, 4)
	c1 <- 100
	c1 <- 200
	c1 <- 300

	// 管道的关闭
	close(c1)

	// 管道关闭后在再读数据
	data := <-c1
	fmt.Println("close channel after, read data:", data)

	// 关闭管道后不能再往管道中写数据
	// c1 <- 400 // 写时报错：panic: send on closed channel
	// fmt.Println("close channel after, write c1:", c1)

	// 管道的遍历
	c2 := make(chan int, 100)
	// 写数据
	for i := 0; i < 100; i++ {
		c2 <- i * 2
	}

	// 遍历管道前未关闭管道，数据能全部遍历完，但报错：fatal error: all goroutines are asleep - deadlock!
	close(c2) // 关闭管道

	// 遍历管道中的数据
	for v := range c2 {
		fmt.Println("Iter channel data:", v)
	}
}

func main() {
	TestChannel()
	TestSaveCompositeType()
	TestChannelCloseIter()
}
