/*
 * @Author: JohnJeep
 * @Date: 2025-04-14 09:37:53
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-04-14 09:55:48
 * @Description:
 * Copyright (c) 2025 by John Jeep, All Rights Reserved.
 */
package main

import "fmt"

func main() {
	ch := make(chan int)

	go func() {
		num := 100
		ch <- num // block
		fmt.Println("child goroutine:", num)
	}()
	fmt.Println("main goroutine chan execute before")
	num := <-ch
	fmt.Println("read data from chan:", num)
}
