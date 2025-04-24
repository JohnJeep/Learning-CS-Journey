/*
 * @Author: JohnJeep
 * @Date: 2025-04-24 15:01:30
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-04-24 16:37:51
 * @Description: 链表
 * Copyright (c) 2025 by John Jeep, All Rights Reserved.
 */
package main

import (
	"errors"
	"fmt"
)

type Node struct {
	data interface{} // 存储任意类型的数据
	next *Node       // 指向下一个节点的指针
}

type LinkedList struct {
	head *Node
	size int
}

func NewLinkedList() *LinkedList {
	return &LinkedList{
		head: nil,
		size: 0,
	}
}

// 链表尾部插入一个新节点
func (ll *LinkedList) Append(data interface{}) {
	newNode := &Node{data: data, next: nil}

	if ll.head == nil {
		ll.head = newNode
	} else {
		current := ll.head
		for current.next != nil {
			current = current.next
		}
		current.next = newNode
	}
	ll.size++
}

// 链表的头部插入一个新节点
func (ll *LinkedList) Prepend(data interface{}) {
	newNode := &Node{data: data, next: ll.head}
	ll.head = newNode
	ll.size++
}

// 指定位置插入一个新节点
func (ll *LinkedList) InsertAt(index int, data interface{}) error {
	if index < 0 || index > ll.size {
		return errors.New("index out of bounds")
	}

	if index == 0 {
		ll.Prepend(data)
		return nil
	}

	newNode := &Node{data: data, next: nil}
	current := ll.head
	for i := 0; i < index-1; i++ {
		current = current.next
	}
	newNode.next = current.next
	current.next = newNode
	ll.size++

	return nil
}

// 删除指定索引的节点
func (ll *LinkedList) DeleteAt(index int) (interface{}, error) {
	if index < 0 || index >= ll.size {
		return nil, errors.New("index out of bounds")
	}

	if index == 0 {
		data := ll.head.data
		ll.head = ll.head.next
		ll.size--
		return data, nil
	}

	current := ll.head
	for i := 0; i < index-1; i++ {
		current = current.next
	}
	data := current.next.data
	current.next = current.next.next
	ll.size--
	return data, nil
}

// 获取指定位置的元素
func (ll *LinkedList) GetAt(index int) (interface{}, error) {
	if index < 0 || index >= ll.size {
		return nil, errors.New("index out of bounds")
	}

	current := ll.head
	for i := 0; i < index; i++ {
		current = current.next
	}
	return current.data, nil
}

// 查找元素第一次出现的位置
func (ll *LinkedList) IndexOf(data interface{}) int {
	current := ll.head
	index := 0
	for current != nil {
		if current.data == data {
			return index
		}
		current = current.next
		index++
	}
	return -1 // not found
}

func (ll *LinkedList) IsEmpty() bool {
	return ll.size == 0
}

func (ll *LinkedList) Size() int {
	return ll.size
}

func (ll *LinkedList) Clear() {
	ll.head = nil
	ll.size = 0
}

func (ll *LinkedList) traverse() {
	current := ll.head
	for current != nil {
		fmt.Printf("%v -> ", current.data)
		current = current.next
	}
	fmt.Println("nil")
}

func main() {
	list := NewLinkedList()
	list.Append(10)
	list.Append(20)
	list.Append(30)
	list.Prepend(5)
	list.InsertAt(2, 15)

	// 5 -> 10 -> 15 -> 20 -> 30 -> nil
	list.traverse()

	fmt.Println("Size:", list.Size())             // Size: 5
	fmt.Println("Index of 20:", list.IndexOf(20)) // Index of 20: 3

	data, err := list.GetAt(2)
	fmt.Println("Get at index 2:", data, "err:", err) // Get at index 2: 15

	data, err = list.DeleteAt(1)
	fmt.Println("Delete at index 1:", data, "err:", err) // Delete at index 1: 10
	list.traverse()                                      // 5 -> 15 -> 20 -> 30 -> nil

	fmt.Println("Size after deletion:", list.Size()) // Size after deletion: 4
	fmt.Println("Is list empty?", list.IsEmpty())    // Is list empty? false

	list.Clear()
	fmt.Println("Size after clear:", list.Size())             // Size after clear: 0
	fmt.Println("Is list empty after clear?", list.IsEmpty()) // Is list empty after clear? true
	list.traverse()                                           // nil
}
