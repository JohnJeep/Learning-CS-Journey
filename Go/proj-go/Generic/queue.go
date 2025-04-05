/*
 * @Author: JohnJeep
 * @Date: 2023-12-04 15:01:13
 * @LastEditors: JohnJeep
 * @LastEditTime: 2023-12-04 15:10:39
 * @Description: 例子：泛型中不能用类型断言或 type swith 来确定接口具体的类型
 *
 * Copyright (c) 2023 by John Jeep, All Rights Reserved.
 */

package main

import (
	"fmt"
	"io"
)

type Queue[T interface{}] struct {
	elements []T
}

// 将数据放入队列尾部
func (q *Queue[T]) Put(value T) {
	q.elements = append(q.elements, value)
}

// !error: 泛型类型定义的变量不能使用类型断言
// func (q *Queue[T]) Put(value T) {
// 	value.(int) // 错误。泛型类型定义的变量不能使用类型断言

// 	// 错误。不允许使用type switch 来判断 value 的具体类型
// 	switch value.(type) {
// 	case int:
// 		// do something
// 	case string:
// 		// do something
// 	default:
// 		// do something
// 	}
// }

// 从队列头部取出并从头部删除对应数据
func (q *Queue[T]) Pop() (T, bool) {
	var value T
	if len(q.elements) == 0 {
		return value, true
	}

	value = q.elements[0]
	q.elements = q.elements[1:]
	return value, len(q.elements) == 0
}

// 队列大小
func (q Queue[T]) Size() int {
	return len(q.elements)
}

// 类型断言动态判断接口的具体类型
func typeAssert() {
	var i interface{} = 10

	switch i.(type) {
	case int:
		fmt.Println("i is int")
	case string:
		fmt.Println("i is string")
	default:
		fmt.Println("i is other type")
	}
}

func main() {
	var q1 Queue[int] // 可存放int类型数据的队列
	q1.Put(1)
	q1.Put(2)
	q1.Put(3)
	fmt.Println("q1: ", q1)
	q1.Pop() // 1
	q1.Pop() // 2
	q1.Pop() // 3

	var q2 Queue[string] // 可存放string类型数据的队列
	q2.Put("A")
	q2.Put("B")
	q2.Put("C")
	fmt.Println("q2: ", q2)
	q2.Pop() // "A"
	q2.Pop() // "B"
	q2.Pop() // "C"

	var q3 Queue[struct{ Name string }]
	fmt.Println("q3: ", q3)

	var q4 Queue[[]int] // 可存放[]int切片的队列
	fmt.Println("q4: ", q4)

	var q5 Queue[chan int] // 可存放int通道的队列
	fmt.Println("q5: ", q5)

	var q6 Queue[io.Reader] // 可存放接口的队列
	fmt.Println("q6: ", q6)
}
