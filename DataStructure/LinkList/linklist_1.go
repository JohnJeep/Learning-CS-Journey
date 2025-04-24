/*
 * @Author: JohnJeep
 * @Date: 2025-04-24 15:02:56
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-04-24 16:39:51
 * @Description: 链表简单实现
 * Copyright (c) 2025 by John Jeep, All Rights Reserved.
 */
package main

import "fmt"

type node struct {
	data int
	next *node
}

func insetNode(prev *node, data int) {
	if prev == nil {
		return
	}
	newNode := &node{data: data, next: prev.next}
	prev.next = newNode // modify pointer of prev to point to new node
}

func traverse(n *node) {
	for n != nil {
		fmt.Printf("%d -> ", n.data)
		n = n.next // move to the next node by pointer
	}
	fmt.Println("nil")
}

// prev：始终指向已经反转部分的链表的头节点
// current：当前正在处理的节点
// next：临时存储当前节点的下一个节点
// 思路：1.改变指针指向
//
//		 2.赋值
//	     3.更新当前值
func reverse(head *node) *node {
	var prev *node
	current := head
	for current != nil {
		next := current.next
		current.next = prev
		prev = current
		current = next
	}
	return prev // new head of the reversed list
}

func main() {
	node1 := &node{data: 10, next: nil}
	node2 := &node{data: 20, next: nil}
	node3 := &node{data: 30, next: nil}

	node1.next = node2
	node2.next = node3

	// traverse: 10 -> 20 -> 30 -> nil
	traverse(node1)

	insetNode(node2, 25)

	// traverse: 10 -> 20 -> 25 -> 30 -> nil
	traverse(node1)

	reversedHead := reverse(node1)
	traverse(reversedHead)
}
