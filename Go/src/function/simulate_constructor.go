// 模拟其他语言中的构造函数
// Go 语言中使用命名约定 NewXXX 来表示构造函数，是一种习惯性命名方式，并不是 Go 语言的强制要求

package main

import "fmt"

type Person struct {
	Name string
	Age  int
}

func NewPerson(name string, age int) *Person {
	p := Person{
		Name: name,
		Age:  age,
	}
	return &p
}

func main() {
	p := NewPerson("Alice", 25)
	fmt.Println(p.Name, p.Age)
}
