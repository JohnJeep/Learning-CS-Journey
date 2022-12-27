package main

import (
	"fmt"
	"runtime"
)

func main() {
	// 返回本地机器的逻辑CPU个数
	cpuNum := runtime.NumCPU()
	fmt.Println("CPU num:", cpuNum)

	// 设置同时可以运行的 CPU 数
	num := runtime.GOMAXPROCS(cpuNum - 1)
	fmt.Println("max procs num:", num)
}
