// 自定义实现错误处理机制

package main

import (
	"errors"
	"fmt"
)

// 正确读配置文件，程序正确执行
// 错误读配置文件，抛出自定义的错误信息，并显示捕获的错误信息，退出程序
func ReadConfig(cfg string) (err error) {
	if cfg == "config.ini" {
		fmt.Println("Read config success ...")
		return nil
	} else {
		return errors.New("Read config failed ...")
	}
}

func TestCustomPainc() {
	if err := ReadConfig("config1.ini"); err != nil {
		// if err := ReadConfig("config.ini"); err != nil {
		panic(err) // 调用内置的 panic 函数，停止并退出程序，后续指令不再执行
	}
	fmt.Println("Program running ...")
}

func main() {
	TestCustomPainc()
}
