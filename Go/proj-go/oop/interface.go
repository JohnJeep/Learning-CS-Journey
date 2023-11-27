package main

import "fmt"

// 定义一个接口
type Usb interface {
	// 待实现的方法
	Start()
	Stop()
}

type Phone struct {
}

// 绑定的方法，传递的是指针类型，高效
func (p *Phone) Start() {
	fmt.Println("Phone start Send msg ..")
}

func (p *Phone) Stop() {
	fmt.Println("Phone stop ...")
}

// Phone 类独有的方法
func (p *Phone) Call() {
	fmt.Println("Phone call ...")
}

type Camera struct {
}

func (c *Camera) Start() {
	fmt.Println("Camera start take a photo ...")
}

func (c *Camera) Stop() {
	fmt.Println("Camera stop ...")
}

type Computer struct {
}

// Working 方法接收接口类型的的变量
func (c *Computer) Working(usb Usb) {
	usb.Start()

	// 使用断言去判断选择调用 Phone 中独有的方法
	if p, ok := usb.(*Phone); ok {
		p.Call() // 等价于 (*p).Call()
	}
	usb.Stop()
}

func main() {
	computer := Computer{}
	IPhone := Phone{}
	sony := Camera{}

	// 调用
	computer.Working(&IPhone)
	computer.Working(&sony)

	// 定义一个数组，数组里面存放Phone和Camera这两个结构体类型的变量
	// 这里就体现了多态数组
	var usbArr [3]Usb
	usbArr[0] = &sony
	usbArr[1] = &IPhone
	usbArr[2] = &sony
	fmt.Println("多态数组:", usbArr)

	for _, v := range usbArr {
		computer.Working(v)
		fmt.Println()
	}

}
