package main

import (
	"factory/mode"
	"fmt"
)

func main() {
	var s = mode.SecondGrade("蔡徐坤", 11)

	fmt.Println("student:", *s)
	fmt.Println("student, name:", s.Name) // (*s).Name 等价于 s.Name 而访问 s.score 发生error
	s.SetScore(88)
	fmt.Println("student Getscore:", s.GetScore())
	fmt.Println("student score change:", *s)
}
