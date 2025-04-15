/*
 * @Author: JohnJeep
 * @Date: 2025-04-15 16:01:39
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-04-15 17:15:20
 * @Description: custom implement hashtable
 * Copyright (c) 2025 by John Jeep, All Rights Reserved.
 */
package main

import (
	"container/list"
)

// Hashtable is a simple implementation of a hash table
// using a linked list to handle collisions
// it uses a slice of linked lists to store the entries
// each entry in the linked list contains a key-value pair
// the key is a string and the value is an int
// the hash table uses a simple hash function to compute the index
type Hashtable struct {
	buckets []*list.List
}

// Entry represents a key-value pair in the hash table
// it contains a key of type string and a value of type int
// it is used to store the entries in the linked list
type Entry struct {
	key   string
	value int
}

func NewHashTable(size int) *Hashtable {
	return &Hashtable{
		buckets: make([]*list.List, size),
	}
}

// hash function to compute the index for a given key
// using a simple hash function for demonstration
func (h *Hashtable) hash(key string) int {
	hash := 0
	for _, c := range key {
		hash = (hash + int(c)) % len(h.buckets)
	}
	return hash
}

// Set adds a key-value pair to the hashtable
// if the key already exists, it updates the value
// if the key does not exist, it creates a new entry
// using a linked list to handle collisions
func (h *Hashtable) Set(key string, value int) {
	index := h.hash(key)
	if h.buckets[index] == nil {
		h.buckets[index] = list.New() // initialize the bucket if it's nil
	}

	// check if the key already exists in the bucket
	for e := h.buckets[index].Front(); e != nil; e = e.Next() {
		entry := e.Value.(*Entry)
		if entry.key == key {
			entry.value = value // update existing entry
			return
		}
	}
	// if key not found, create a new entry
	newEntry := &Entry{key: key, value: value}
	h.buckets[index].PushBack(newEntry)
}

// Get retrieves the value for a given key
// returns the value and a boolean indicating if the key was found
// if the key is not found, returns 0 and false
func (h *Hashtable) Get(key string) (int, bool) {
	index := h.hash(key) // compute the index
	if h.buckets[index] != nil {
		// search for the key in the bucket
		// using a linked list to handle collisions
		// iterate through the linked list
		// to find the entry with the matching key
		// if found, return the value
		// if not found, return nil
		for e := h.buckets[index].Front(); e != nil; e = e.Next() {
			entry := e.Value.(*Entry) // type assertion to get the Entry
			if entry.key == key {
				return entry.value, true
			}
		}
	}
	return 0, false // not found
}

func main() {
	// Example usage
	hashTable := NewHashTable(10)
	hashTable.Set("key1", 1)
	hashTable.Set("key2", 2)

	value, found := hashTable.Get("key1")
	if found {
		println("Found key1:", value)
	} else {
		println("key1 not found")
	}

	value, found = hashTable.Get("key3")
	if found {
		println("Found key3:", value)
	} else {
		println("key3 not found")
	}
	hashTable.Set("key1", 10) // update key1
	value, found = hashTable.Get("key1")
	if found {
		println("Updated key1:", value)
	} else {
		println("key1 not found")
	}
	hashTable.Set("key2", 20) // update key2
	value, found = hashTable.Get("key2")
	if found {
		println("Updated key2:", value)
	} else {
		println("key2 not found")
	}
}
