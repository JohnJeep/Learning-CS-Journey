package main

import (
	"fmt"
	"math/rand"
	"time"
)

func main() {
	// Create a new random number generator with a custom seed
	source := rand.NewSource(time.Now().UnixNano())
	rand := rand.New(source)

	// Generate a random number of minutes between 1 and 15
	randMinutes := rand.Intn(15) + 1
	fmt.Println("randMinutes:", randMinutes)
}
