// time标准库中的time.NewTicker()函数返回一个带有channel的结构体，
// 并定时向这个结构体中发送时间数据，以实现定时器的功能。

package main

import (
	"fmt"
	"time"
)

func main() {
	c := time.NewTicker(time.Second)
	for t := range c.C {
		fmt.Printf("receive t :%s\n", t)
	}
}