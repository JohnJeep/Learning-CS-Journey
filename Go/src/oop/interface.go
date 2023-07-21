/*
 * @Author: JohnJeep
 * @Date: 2023-01-06 17:48:37
 * @LastEditors: JohnJeep
 * @LastEditTime: 2023-07-11 14:37:27
 * @Description: How to use interface in Golang  
 * Copyright (c) 2023 by John Jeep, All Rights Reserved. 
 */

package main

import "fmt"

// 定义一个接口
// 里面是类待实现的方法
type Usb interface {
	Start()
	Stop()
}

// 空类，没有定义成员变量
type Phone struct {}

// 绑定的方法，传递的是指针类型，高效
// Phone 类实现了接口中的两个方法
func (p *Phone) Start() {
	fmt.Println("Phone start Send msg ..")
}

func (p *Phone) Stop() {
	fmt.Println("Phone stop ...")
}

// Phone 类独有的方法
func (p *Phone) Call() {
	fmt.Println("Phone call ...")
}iPhone

// 空类，没有定义成员变量
type Camera struct {}

// Camera 类实现了接口中的两个方法
func (c *Camera) Start() {
	fmt.Println("Camera start take a photo ...")
}

func (c *Camera) Stop() {
	fmt.Println("Camera stop ...")
}

// 空类，没有定义成员变量
type Computer struct {}

// Computer 类的 Working 方法负责去处理接口中的方法
// 传入的参数为接口类型的的变量
func (c *Computer) Working(usb Usb) {
	usb.Start()

	// 使用断言去判断选择调用 Phone 中独有的方法
	if p, ok := usb.(*Phone); ok {
		p.Call() // 等价于 (*p).Call()
	}
	usb.Stop()
}

func main() {
	// 定义对象
	computer := Computer{}
	iPhone := Phone{}
	sony := Camera{}

	// 调用
	computer.Working(&iPhone)
	computer.Working(&sony)

	// 定义一个数组，数组里面存放Phone和Camera这两个结构体类型的变量
	// 这里就体现了多态数组
	var usbArr [3]Usb
	usbArr[0] = &sony
	usbArr[1] = &iPhone
	usbArr[2] = &sony
	fmt.Println("多态数组:", usbArr)

	for _, v := range usbArr {
		computer.Working(v)
		fmt.Println()
	}

}
