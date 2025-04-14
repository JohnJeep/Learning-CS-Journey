package main

import "fmt"

func main() {
	ch := make(chan int, 2)

	go func() {
		ch <- 1
		ch <- 2
		fmt.Println("child goroutine ...")
	}()

	v1 := <-ch
	v2 := <-ch
	fmt.Println("read data from chan:", v1, v2)
}
