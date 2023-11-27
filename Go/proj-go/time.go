package main

import "fmt"

/*
时间常量
*/
const (
	SecondsPerMinute = 60                    //定义每分钟的秒数
	SecondsPerHour   = SecondsPerMinute * 60 //定义每小时的秒数
	SecondsPerDay    = SecondsPerHour * 24   //定义每天的秒数
)

/*
时间转换函数
*/
func resolveTime(seconds int) (day int, hour int, minute int) {
	minute = seconds / SecondsPerMinute //每分钟秒数
	hour = seconds / SecondsPerHour     //每小时秒数
	day = seconds / SecondsPerDay       //每天秒数
	return
}

func main() {
	//打印返回参数
	fmt.Println(resolveTime(1000))

	//只打印小时和分钟
	_, hour, minute := resolveTime(359940)
	fmt.Println(hour, minute)

	//只打印天
	day, _, _ := resolveTime(90000)
	fmt.Println(day)
}
