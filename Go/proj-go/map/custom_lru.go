/*
 * @Author: JohnJeep
 * @Date: 2025-04-17 15:02:45
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-04-17 18:26:25
 * @Description: 自定义双向链表实现LRU缓存
 * Copyright (c) 2025 by John Jeep, All Rights Reserved.
 */

package main

import "fmt"

type Node struct {
	Key   int
	Value int
	Prev  *Node
	Next  *Node
}

type LRUCache struct {
	capacity int
	size     int
	cache    map[int]*Node
	head     *Node
	tail     *Node
}

func Constructor(capacity int) LRUCache {
	lru := LRUCache{
		capacity: capacity,
		cache:    make(map[int]*Node),
		head:     &Node{},
		tail:     &Node{},
	}
	lru.head.Next = lru.tail // head's next is tail
	lru.tail.Prev = lru.head // tail's prev is head
	return lru
}

// Get retrieves the value of the key if the key exists in the cache.
// If the key does not exist, it returns -1.
// It also moves the accessed node to the head of the list (most recently used).
func (l *LRUCache) Get(key int) int {
	if node, exists := l.cache[key]; exists {
		l.removeNode(node)
		l.addToHead(node)
		return node.Value
	}
	return -1
}

// Put adds a key-value pair to the cache.
// If the key already exists, it updates the value and moves the node to the head.
// If the key does not exist and the cache is full, it removes the least recently used node.
func (l *LRUCache) Put(key, value int) {
	if node, exists := l.cache[key]; exists {
		node.Value = value
		l.removeNode(node)
		l.addToHead(node)
	} else {
		newNode := &Node{Key: key, Value: value}
		l.cache[key] = newNode
		l.addToHead(newNode)
		l.size++
		if l.size > l.capacity {
			removed := l.removeTail()
			delete(l.cache, removed.Key)
			l.size--
		}
	}
}

// addToHead adds a node to the head of the doubly linked list.
// It is assumed that the node is not already in the list.
// The head and tail are dummy nodes.
// The head's next is the first real node, and the tail's prev is the last real node.
// The head and tail are not counted in the size of the list.
func (l *LRUCache) addToHead(node *Node) {
	node.Prev = l.head      // head's next is the first real node
	node.Next = l.head.Next // the first real node's prev is the new node
	l.head.Next.Prev = node // the new node's next is the first real node
	l.head.Next = node      // head's next is the new node
}

// removeNode removes a node from the doubly linked list.
// It does not delete the node from the cache.
// This is useful when we want to move the node to the head.
// It is assumed that the node is not the head or tail.
// The head and tail are dummy nodes.
// The head's next is the first real node, and the tail's prev is the last real node.
// The head and tail are not counted in the size of the list.
func (l *LRUCache) removeNode(node *Node) {
	node.Prev.Next = node.Next // the previous node's next is the next node
	node.Next.Prev = node.Prev // the next node's prev is the previous node
	node.Prev = nil            // clear the node's prev pointer
	node.Next = nil            // clear the node's next pointer
}

// removeTail removes the least recently used node (tail's previous node)
// and returns it. If the list is empty, it returns nil.
func (l *LRUCache) removeTail() *Node {
	if l.tail.Prev == l.head {
		return nil
	}
	tailNode := l.tail.Prev
	l.removeNode(tailNode)
	return tailNode
}

func main() {
	// Example usage
	lru := Constructor(2)
	lru.Put(1, 1)
	lru.Put(2, 2)
	fmt.Println(lru.Get(1)) // returns 1
	lru.Put(3, 3)           // evicts key 2
	fmt.Println(lru.Get(2)) // returns -1 (not found)
	lru.Put(4, 4)           // evicts key 1
	fmt.Println(lru.Get(1)) // returns -1 (not found)
	fmt.Println(lru.Get(3)) // returns 3
	fmt.Println(lru.Get(4)) // returns 4
	lru.Put(5, 5)           // evicts key 3
	fmt.Println(lru.Get(3)) // returns -1 (not found)
	fmt.Println(lru.Get(4)) // returns 4
	fmt.Println(lru.Get(5)) // returns 5
}
