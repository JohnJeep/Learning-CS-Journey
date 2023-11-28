package main

import "fmt"

// 在进入main入口函数之前，每个init函数在此包加载的时候将被（串行）执行并且只执行一遍
// init 函数执行的顺序与代码中的位置有关
func init() {
	fmt.Println("second init")
}

func init() {
	fmt.Println("first init")
}

func main() {
	fmt.Println("exceute main")
}
