// Description: time包提供了时间的显示和测量用的函数。日历的计算采用的是公历。
// time包中定义的时间类型Time代表一个时间点。内部保存了秒和纳秒两个整数，从而可以保证时间的操作具有一定的精度。
// time包中定义的Duration类型代表两个时间点之间经过的时间，以纳秒为单位。
// time包中定义的Ticker类型代表一个间隔时间定时器，它会以一个间隔时间往Channel发送一个事件。
// time包中定义的Timer类型代表一个单次时间定时器，它会在一个设定的时间往Channel发送一个事件。
// 
// time.Now() 函数用于获取当前的本地时间（按系统时区），返回一个 time.Time 类型的对象，表示当前的时间点。
// time.Now().Year() 返回当前年份
// time.Now().Month() 返回当前月份
// time.Now().Day() 返回当前日
// time.Now().Hour() 返回当前小时
// time.Now().Minute() 返回当前分钟
// time.Now().Second() 返回当前秒
// time.Now().Nanosecond() 返回当前纳秒
// time.Now().Weekday() 返回当前星期几
// time.Now().YearDay() 返回当前年份中的第几天
// time.Now().Unix() 返回当前时间的 Unix 时间，即从 1970 年 1 月 1 日至今的秒数
// time.Now().UnixNano() 返回当前时间的 Unix 时间，即从 1970 年 1 月 1 日至今的纳秒数
// time.Now().Format() 函数用于格式化时间，返回一个格式化的字符串
// time.Now().Add() 函数用于时间的加减，返回一个时间
// time.Now().Sub() 函数用于时间的减法，返回一个时间间隔
// time.Now().Before() 函数用于判断当前时间是否在另一个时间之前
// time.Now().After() 函数用于判断当前时间是否在另一个时间之后
// time.Now().Equal() 函数用于判断两个时间是否相等
// time.Now().Truncate() 函数用于截断时间，返回一个时间
// time.Now().Round() 函数用于四舍五入时间，返回一个时间
// time.Now().Location() 函数用于获取当前时间的时区
// time.Now().In() 函数用于设置当前时间的时区，返回一个时间
// time.Now().Local() 函数用于获取当前时间的本地时间
// time.Now().UTC() 函数用于获取当前时间的 UTC 时间
// time.Now().UnixDate() 函数用于格式化时间，返回一个格式化的字符串


// time.Unix(sec int64, nsec int64) 函数用于根据给定的秒数和纳秒数创建一个对应的 time.Time 对象，
// 表示从 Unix 纪元（1970-01-01 00:00:00 UTC）开始经过的时间。


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
