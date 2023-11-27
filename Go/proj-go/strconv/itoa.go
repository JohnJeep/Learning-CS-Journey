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
