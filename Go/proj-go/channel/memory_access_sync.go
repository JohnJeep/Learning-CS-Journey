/*
 * @Author: JohnJeep
 * @Date: 2023-12-04 15:45:24
 * @LastEditors: JohnJeep
 * @LastEditTime: 2023-12-04 18:26:02
 * @Description:
 * Copyright (c) 2023 by John Jeep, All Rights Reserved.
 */
package main

import "fmt"

var data int

func main() {
	go func() { data++ }()
	if data == 0 {
		fmt.Println("the value is 0.")
	} else {
		fmt.Printf("the value is %v.\n", data)
	}
}
