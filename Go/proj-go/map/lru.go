/*
 * @Author: JohnJeep
 * @Date: 2025-04-17 00:15:43
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-04-17 00:24:11
 * @Description: LRU Cache implementation by using a doubly linked list and a map in Go,
 *               which is a common approach to implement an LRU cache.
 * Copyright (c) 2025 by John Jeep, All Rights Reserved.
 */

package main

import (
	"container/list"
	"fmt"
)

type LRUCache struct {
	capacity int
	cache    map[int]*list.Element
	list     *list.List
}

type entry struct {
	key   int
	value int
}

func Constructor(capacity int) LRUCache {
	return LRUCache{
		capacity: capacity,
		cache:    make(map[int]*list.Element),
		list:     list.New(),
	}
}

func (this *LRUCache) Get(key int) int {
	if elem, ok := this.cache[key]; ok {
		this.list.MoveToFront(elem)
		return elem.Value.(*entry).value
	}
	return -1
}

func (this *LRUCache) Put(key int, value int) {
	if elem, ok := this.cache[key]; ok {
		// elem.Value is interface{}, so we need to type assert it to *entry
		// to access the key and value fields
		// elem.Value.(*entry).key = key // update the key (not necessary)
		// elem.Value.(*entry).value = value // update the value
		elem.Value.(*entry).value = value // update the value
		this.list.MoveToFront(elem)       // move the accessed element to the front
	} else {
		if this.list.Len() >= this.capacity {
			// Remove the least recently used entry
			tail := this.list.Back()
			if tail != nil {
				delete(this.cache, tail.Value.(*entry).key) // remove from cache
				this.list.Remove(tail)                      // remove from list
			}
		}
		// Add new entry to the front
		newEntry := &entry{key: key, value: value} // create new entry
		elem := this.list.PushFront(newEntry)      // add to list
		this.cache[key] = elem                     // add to cache
	}
}

func main() {
	// Example usage
	cache := Constructor(2)
	cache.Put(1, 1)
	cache.Put(2, 2)
	fmt.Println(cache.Get(1)) // returns 1
	cache.Put(3, 3)           // evicts key 2
	fmt.Println(cache.Get(2)) // returns -1 (not found)
	cache.Put(4, 4)           // evicts key 1
	fmt.Println(cache.Get(1)) // returns -1 (not found)
	fmt.Println(cache.Get(3)) // returns 3
	fmt.Println(cache.Get(4)) // returns 4
}
