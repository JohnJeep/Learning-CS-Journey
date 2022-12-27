// 结构体成员分为导出和未导出成员，结构体成员名字是以大写字母开头的，那么该成员就是导出的
// 两个相同类型的结构体，结构体成员的顺序不同，结构体也不同

package main

import (
	"fmt"
	"time"
)

// 声明结构体类型
type Employee struct {
	ID            int
	Name, Address string
	Dob           time.Time
	Position      string
	Salary        int
	ManagerID     int
	Number        [5]int
	slice         []int
	mp            map[string]int
	ptr           *int
}

// 常见结构体实例化的四种方式
func InstanceStruct() {

	// 方式一：直接声明一个类型为 Employee 的结构体变量
	var emp1 Employee
	fmt.Println("emp1:", emp1)

	// 方式二：创建时直接赋值，类型名可省略
	var emp2 Employee = Employee{ID: 999, Name: "Jack", Address: "88434"}
	fmt.Println("emp2:", emp2)

	// 考虑效率的话， 较大的结构体通常会用指针的方式传入和返回

	// 方式三：使用 new 的方式
	var emp3 *Employee = new(Employee) // 等价于 emp3 := new(Employee)
	emp3.ID = 555                      // 与 (*emp3).ID = 556 这种写法等价，编译器底层对了进行优化，加上了 * 操作
	(*emp3).ID = 556                   // 指针成员赋值标准写法，但有点繁琐
	emp3.ManagerID = 10089
	fmt.Println("emp3:", emp3)

	// 采用 & 取地址的方式，创建结构体实例并初始化
	emp4 := &Employee{Position: "Shenzhen", Salary: 99999} // 这种初始化，不用关心属性的顺序
	fmt.Println("emp4:", emp4)
}

// 结构体用 type 重定义
func TestStructType() {
	type Teacher struct {
		ID   int
		Name string
	}

	type YuanDing Teacher // Golang 认为 YuanDing 是一种新的数据类型
	var t1 Teacher
	var y1 YuanDing
	// t1 = y1 // y1 与 t1 之间不能互转
	t1 = Teacher(y1) // 强转

	fmt.Println("t1:", t1, "y1:", y1)
}

func TestStructInit() {
	var emp Employee

	// 打印结构体的初值
	fmt.Println("Struct 默认零值:", emp)

	// 判断 slice、map、指针的默认零值是否为 nil
	if emp.slice == nil {
		fmt.Println("slice 默认零值是 nil")
	}
	if emp.mp == nil {
		fmt.Println("map 默认零值是 nil")
	}
	if emp.ptr == nil {
		fmt.Println("指针ptr默认零值是 nil")
	}

	emp.ID = 007 // 结构体成员赋值

	// 成员取地址，指针访问
	pos := &emp.Position
	*pos = "Beijin"
	fmt.Println("pos:", *pos)

	// ptr 指针
	emp.ptr = &emp.ID

	// 给 slice、map 成员赋值
	emp.slice = make([]int, 3)
	emp.slice[2] = 999

	emp.mp = make(map[string]int)
	emp.mp["部门"] = 888
	fmt.Println("emloyee info:", emp)
}

func main() {
	TestStructInit()
	TestStructType()
	InstanceStruct()
}
