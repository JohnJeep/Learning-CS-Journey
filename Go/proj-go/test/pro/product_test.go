package pro

import (
	"testing"
)

// 普通测试函数
// 函数名必须以 Test 前缀，形参类型必须为 *testing.T
func TestBuy(t *testing.T) {
	p1 := Product{"牛奶", 5.5, 25}
	p2 := Product{"鸡翅", 12.8, 5}
	adult := Person{"张三"}

	adult.Buy(&p1, &p2)
}

// 基准测试函数
// 函数名前缀必须为 Benchmark，形参类型必须为 *testing.B
func BenchmarkBuy(b *testing.B) {
	for i := 0; i < 10; i++ {
		bp1 := Product{"英语书", 99, 5}
		bp2 := Product{"钢笔", 28, 10}

		teacher := Person{"李四"}
		teacher.Buy(&bp1, &bp2)
	}
}
