// if 后面的条件表达式，省略了括号
// 代码块左 括号必须在条件表达式尾部
// 持初始化语句，可定义代码块局部变量

package main

import (
	"fmt"
)

func main() {
	// if 中的条件表达式，可以有一个初始值变量，后面再跟判断语句
	if num := 95; num < 60 {
		fmt.Println("num大于60, num:", num)
	} else if num > 80 {
		fmt.Println("num大于80, num", num)
	} else if num >= 90 && num < 100 {
		fmt.Println("num: 90~100, num", num)
	}

	// fmt.Println(num)  变量num只在 if 条件语句的作用域类有效
}
