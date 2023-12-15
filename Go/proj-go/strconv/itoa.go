/*
 * @Author: JohnJeep
 * @Date: 2022-12-04 10:19:49
 * @LastEditors: JohnJeep
 * @LastEditTime: 2023-12-04 15:11:09
 * @Description: Itoa 将 10 进制数转换为字符串
 * Copyright (c) 2023 by John Jeep, All Rights Reserved.
 */
package main

import (
	"fmt"
	"strconv"
)

func main() {
	val := 100
	s := strconv.Itoa(val) // 10 进制数转 string

	fmt.Printf("type: %T, value: %v\n", s, s)
}
