/*
 * @Author: JohnJeep
 * @Date: 2023-12-04 11:40:17
 * @LastEditors: JohnJeep
 * @LastEditTime: 2023-12-04 14:54:20
 * @Description: Go 泛型使用
 *               Go 1.18版本增加了对泛型的支持
 *               Go 1.18引入了一个新的预声明标识符 any，作为空接口类型的别名
 *
 *               基本概念
 *                   类型形参(Type parameter)
 *                   类型实参(Type argument)
 *                   类型形参列表(Type parameter list)
 *                   类型约束(Type constraint)
 *                   实例化(Instantiations)
 *                   泛型类型(Generic type)
 *                   泛型接收器(Generic receiver)
 *                   泛型函数(Generic function)
 *
 *               符号说明
 *                   |：表示或，用于分隔类型约束，表示类型约束的并集
 *                   ~: 表示非，用于分隔类型约束，表示类型约束的补集；~ 符号后的类型只能是基本类型，不能是接口类型 interface
 *
 *                   注意：
 *                   1、泛型函数的类型形参列表必须放在函数名之后
 *                   2、匿名函数中不能使用泛型，特别是在做单元测试用例时，需要注意
 *                   3、使用接口的时候经常会用到类型断言或 type swith 来确定接口具体的类型，然后对不同类型做出不同的处理
 *                      而使用泛型类型定义的变量，不能使用类型断言或 type swith 来确定具体的类型
 *                   4、目前不是支持泛型方法
 *
 *               引入泛型的作用
 *                   泛型的引入是为了配合接口的使用，让我们能够编写更加类型安全的Go代码，并能有效地减少重复代码。
 *
 * @References: https://www.liwenzhou.com/posts/Go/generics/
 * @References：https://segmentfault.com/a/1190000041634906
 *
 * Copyright (c) 2023 by John Jeep, All Rights Reserved.
 */
package main

import (
	"fmt"
)

func AddInt(x, y int) int {
	return x + y
}

func AddFloat(x, y float64) float64 {
	return x + y
}

func AddString(x, y string) string {
	return x + y
}

// Add 为泛型函数
// T 为类型参数(type parameter)，可以是 int、float64、string
// 类型参数列表： [T int | float64 | string
// 类型约束(type constraint)： int、float64、string
func Add[T int | float64 | string](x, y T) T {
	return x + y
}

func main() {
	fmt.Println(AddInt(10, 20))
	fmt.Println(AddFloat(10.2, 20.3))
	fmt.Println(AddString("hello ", "world"))

	fmt.Println(Add[int](20, 30))             // 显式指定类型实参 int
	fmt.Println(Add[float64](40.2, 60.3))     // 显式指定类型实参 float64
	fmt.Println(Add[string]("good ", "noce")) // 显式指定类型实参 string
}
