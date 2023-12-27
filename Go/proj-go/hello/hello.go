package main

import (
	"fmt"
	"time"
)

func main() {
	var s string
	fmt.Println("nil str:", s)
	fmt.Println("nil str len:", len(s))
	fmt.Printf("Hello Word!\n")
	fmt.Println("Ni hao!")
	fmt.Println("time: ", time.Now())
	fmt.Println("Unix time: ", time.UnixDate)
}
