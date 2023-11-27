package main

import (
	"fmt"
	"reflect"
)

// 基本数据类型、空接口 interface{}、reflect.Value 三者反射之间的数据转换
func Convert(i interface{}) {
	// 查看传入接口中的类型
	rType := reflect.TypeOf(i)
	fmt.Println("rTyrpe:", rType)

	// 空接口转reflect.Value
	val := reflect.ValueOf(i)
	fmt.Printf("val type: %T, value: %v\n", val, val)

	// 反射内部，对 reflect.Value 类型的数据进行操作，返回普通的基础类型
	dat := val.Int() + 1000 // 转化为 int
	fmt.Printf("dat type: %T, value: %v\n", dat, dat)

	// reflect.Value转空接口
	anyVal := val.Interface()
	fmt.Printf("anyValue type: %T, value: %v\n", anyVal, anyVal)

	// interface{} 转基础类型
	baseVal := anyVal.(int)
	fmt.Printf("baseVal type: %T, value: %v\n", baseVal, baseVal)
}

func Feflect01() {
	num := 100
	Convert(num) // 传值时将基础类型转化为空接口
}

// 结构体类型、空接口 interface{}、reflect.Value 三者反射之间的数据转换
type Student struct {
	Name  string
	Age   int
	Score int
}

func ConvertComposite(i interface{}) {
	// 空接口转 reflect.Value
	rVal := reflect.ValueOf(i)
	fmt.Printf("Struct reflect value type: %T, value: %v\n", rVal, rVal)

	// reflect.Value 转空接口
	anyVal := rVal.Interface()
	fmt.Printf("Struct any type: %T, value: %v\n", anyVal, anyVal)

	// 类型断言
	structVal, ok := anyVal.(Student)
	if ok {
		fmt.Printf("Struct data type: %T, name: %s, age: %d, score: %d\n",
			structVal, structVal.Name, structVal.Age, structVal.Score)
	}
}

func FelectStruct() {
	stu := Student{Name: "Mike", Age: 18, Score: 100}
	ConvertComposite(stu)
}

func main() {
	Feflect01()
	FelectStruct()
}
