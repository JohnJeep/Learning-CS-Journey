// select 语句类似于 switch 语句，但是select会随机执行一个可运行的case。如果没有case可运行，它将阻塞，直到有case可运行。
// select 是Go中的一个控制结构，类似于用于通信的switch语句。每个case必须是一个通信操作，要么是发送要么是接收。 select 随机执行一个可运行的case。
// 如果没有case可运行，它将阻塞，直到有case可运行。一个默认的子句应该总是可运行的。

package main

import "fmt"

func main() {

	fmt.Println()

}
