// switch 分支中默认省略了break
// fallthrough: 关键字表示不执行break，与break的意义相反
// switch 后面的变量与case后面的变量，必须是相同的类型获知最终结果为相同类型的表达式

package main

import "fmt"

func main() {
	// score := 100
	var score int
	fmt.Printf("请输入分数：")
	fmt.Scan(&score)

	switch { // switch 后面的变量可以省略，case 中的值可以为任意的表达式，不限于常量
	case score == 100:
		fmt.Println("天才, score:", score)
	case score >= 80 && score < 100:
		fmt.Println("优秀, score:", score)
	case score >= 70 && score < 80:
		fmt.Println("良好, score:", score)
	case score >= 60 && score < 70:
		fmt.Println("合格, score:", score)
	default:
		fmt.Println("不及格, score:", score)
	}

}
