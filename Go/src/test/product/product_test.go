package pro

import (
	"testing"
)

func TestBuy(t *testing.T) {
	p1 := Product{"牛奶", 5.5, 25}
	p2 := Product{"鸡翅", 12.8, 5}
	adult := Person{"张三"}

	adult.Buy(&p1, &p2)
}
