// 方法的声明和调用

package main

import "fmt"

type Person struct {
	Name string
}

// 声明一个方法；MethodPerson 方法绑定于 Person结构体类型
// person 是按照值传递的，是 main 函数中 p 的一个副本
func (person Person) MethodPerson() {
	person.Name = "Tim"
	fmt.Println("方法内部, Name:", person.Name)
}

// 按照地址传递
func (person *Person) MethodPersonAddr() {
	person.Name = "Google"
	fmt.Println("方法内部, Name:", person.Name)
}

// 重写 String() 方法
type Student struct {
	Name  string
	Score int
}

func (stu *Student) String() string {
	str := fmt.Sprintf("Name=%s, Score=%d", stu.Name, stu.Score)
	return str
}

func main() {
	p := Person{"Jack"}
	p.MethodPerson() // 调用方法；
	fmt.Println("方法外部, Name:", p.Name)
	(&p).MethodPerson() // 这种也是可以的，从形式看是按照地址传递的，但实际上是按照值传递的。因为Go编译器底层做了优化，就变成了 p.MethodPerson()
	fmt.Println("方法外部 &p, Name:", p.Name)

	p1 := Person{"Apple"}
	p1.MethodPersonAddr() // 调用方法，p1 从形式上看是传递的是值，但实际上传递的是地址，等价于(&p1).MethodPersonAddr
	fmt.Println("方法外部 p1, Name:", p1.Name)

	// 调用重写的String
	s := Student{
		Name:  "小明",
		Score: 100,
	}
	fmt.Println(s)  // 调用默认的String方法
	fmt.Println(&s) // &s 取对象实例的地址
}
