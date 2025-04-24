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

// Usb interface 作为函数的参数
// interface 也可以作为函数的返回值、也可作为 struct 的字段，实现依赖注入和松耦合
func (c *Computer) Working(usb Usb) {
	usb.Start()

	// 使用断言去判断选择调用 Phone 中独有的方法
	if p, ok := usb.(*Phone); ok {
		p.Call() // 等价于 (*p).Call()
	}
	usb.Stop() // usb 是一个接口类型，运行时会根据传入的参数类型动态来调用相应的方法
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
