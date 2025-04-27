/*
 * @Author: JohnJeep
 * @Date: 2025-04-27 16:41:02
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-04-27 16:51:04
 * @Description: flag cmd parse usage
 * Copyright (c) 2025 by John Jeep, All Rights Reserved.
 */
package main

import (
	"flag"
	"fmt"
)

// go run flag.go --name peter --age 18 --verbose true
// name: 命令参数
// value: 外部没有传入，直接使用函数中的默认值
// usage: 命令参数的描述
// flag.Parse() 解析命令行参数
func main() {
	name := flag.String("name", "John", "a name to say hello to")
	age := flag.Int("age", 0, "your age")
	verbose := flag.Bool("verbose", false, "enable verbose output")
	flag.Parse()

	fmt.Println("name:", *name, "age:", *age, "verbose:", *verbose)
}
