// Goja: A Golang JavaScript Runtime

package main

import (
	"fmt"

	"github.com/dop251/goja"
)

type Person struct {
	Name string
	age  int
}

func (p *Person) GetAge() int {
	return p.age

}
func main() {
	p := &Person{
		Name: "John",
		age:  30,
	}
	vm := goja.New()

	// Set the Person struct in the JavaScript runtime
	vm.Set("person", p)

	// JavaScript code to access the struct's fields and methods
	script := `
		const name = person.Name;
		const age = person.GetAge();
		name + " is " + age + " years old.";
	`
	result, err := vm.RunString(script)
	if err != nil {
		fmt.Println("Error:", err)
		return
	}
	fmt.Printf("result:%v, type:%T\n", result, result)                   // Output: John is 30 years old.
	fmt.Printf("result:%v, type:%T\n", result.String(), result.String()) // Output: John is 30 years old.
}
