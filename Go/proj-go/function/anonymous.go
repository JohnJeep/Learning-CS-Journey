// 匿名函数与闭包用法
// 匿名函数：没有名字的函数

package fucntion

import "fmt"

func Anonymous() {
	id := 0o07
	name := "Mike"
	// 定义匿名函数，用一个变量去接匿名函数的结果
	f := func() {
		fmt.Println("Anonymous...")
		fmt.Println("id:", id, "name:", name) // 匿名函数以引用的方式捕获的外部变量 id，name
	}
	// 调用匿名函数
	f()

	// 定义匿名函数的同时直接调用
	// 函数定义完后，在后面用 () 表示直接调用匿名函数
	func() {
		fmt.Println("Declartion anonymous and call...")
	}()

	// 匿名函数带参数和返回值
	func(x, y int) (min, max int) {
		if x > y {
			min = y
			max = x
		} else {
			min = x
			max = y
		}
		fmt.Printf("min = %d, max = %d\n", min, max)
		return
	}(100, 50)

	// 带有返回值和参数的匿名函数，用一个变量去接函数的返回的值
	s := func(m1, m2 int) (sum int) {
		sum = m1 + m2
		return
	}(40, 20)
	// fmt.Printf("sum = %d, dev = %d\n", s, d)
	fmt.Printf("sum = %d\n", s)

	// 多个参数，多个返回值
	s2, dev := func(m1, m2 int) (sum, dev int) {
		sum = m1 + m2
		dev = m1 - m2
		return
	}(200, 100)
	fmt.Printf("sum = %d, dev = %d\n", s2, dev)
}
