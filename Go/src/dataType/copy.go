package main

import "fmt"

func main() {
	src := []int{4, 5, 6}
	dest := []int{0, 0, 0, 0, 0, 0}
	copy(dest, src)

	fmt.Println("dest:", dest)
}
