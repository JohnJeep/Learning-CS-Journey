package main

import (
	"fmt"
	"strings"
	"time"
)

func builder() {
	var builder strings.Builder
	// Preallocate memory for the builder to avoid multiple allocations
	builder.Grow(100)

	// Write strings to the builder
	builder.WriteString("This is a builder string")
	builder.WriteString(" and it is very useful.")
	builder.WriteString(" It is more efficient than using string concatenation.")

	// Write a formatted string
	result := builder.String()
	fmt.Println("Result:", result)
}

func concatWithPlus() string {
	s := ""
	for i := 0; i < 100000; i++ {
		s += "a"
	}
	return s
}

func concatWithBuilder() string {
	var builder strings.Builder
	for i := 0; i < 100000; i++ {
		builder.WriteString("a")
	}
	return builder.String()
}

func main() {
	// builder()

	start := time.Now()
	concatWithPlus()
	elapsed := time.Since(start)
	fmt.Println("Time taken with +:", elapsed)

	start = time.Now()
	concatWithBuilder()
	elapsed = time.Since(start)
	fmt.Println("Time taken with strings.Builder:", elapsed)
}
