package main

import (
	_ "fmt"
)

func main() {
	paomian := pro.Product{"泡面", 4.5, 6}
	hotDog := pro.Product{"热狗", 2.5, 14}

	person := pro.Person{"小芳"}
	person.Buy(&paomian, &hotDog)
}
