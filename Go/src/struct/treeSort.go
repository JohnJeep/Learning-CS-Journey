// 二叉树实现插入排序

package main

import (
	"fmt"
)

type tree struct {
	value       int
	left, right *tree // 结构体中包含 *tree，创建递归数据结构
}

func Sort(values []int) {
	var root *tree
	for _, v := range values {
		root = add(root, v)
	}
	appendValues(values[:0], root)
}

func add(t *tree, val int) *tree {
	if t == nil {
		t = new(tree)
		t.value = val
		return t
	}

	if val < t.value {
		t.left = add(t.left, val)
	} else {
		t.right = add(t.right, val)
	}
	return t
}

func appendValues(val []int, t *tree) []int {
	if t != nil {
		val = appendValues(val, t.left)
		val = append(val, t.value)
		val = appendValues(val, t.right)
	}
	return val
}

func main() {
	var s = []int{5, 7, 8, 0, 3} // 定义一个slice并赋初值
	Sort(s)
	fmt.Println("Binary tree insert sort:", s)

}
