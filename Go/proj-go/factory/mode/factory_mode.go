// 利用工厂模式跨包创建结构体的实例
// go 中没有构造函数，利用工厂模式实现类似构造函数

package mode

type student struct {
	Name  string
	score int // 不能导出的字段，相当于是私有的
}

// 定义一个可导出的函数，让外部的包可访问结构体中可导出的字段；相当于一个工厂模式
func SecondGrade(n string, s int) *student {
	return &student{
		Name:  n,
		score: s,
	}
}

// 给私有变量绑定一个方法，外部的包通过方法间接访问结构体中的变量
func (stu *student) GetScore() int {
	return stu.score
}

func (stu *student) SetScore(sc int) {
	stu.score = sc
}
