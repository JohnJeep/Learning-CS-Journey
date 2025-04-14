/*
 * @Author: JohnJeep
 * @Date: 2025-04-14 17:14:17
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-04-14 18:15:31
 * @Description: deadlock usage
 * Copyright (c) 2025 by John Jeep, All Rights Reserved.
 */
package main

import (
	"os"
	"runtime/trace"
	"sync"
)

func main() {
	f, err := os.Create("trace.out")
	if err != nil {
		panic(err)
	}
	defer f.Close()

	err = trace.Start(f)
	if err != nil {
		panic(err)
	}
	defer trace.Stop()

	var mu1, mu2 sync.Mutex
	var wg sync.WaitGroup

	wg.Add(2)

	go func() {
		defer wg.Done()
		mu1.Lock()
		mu2.Lock()
		mu2.Unlock()
		mu1.Unlock()
	}()

	go func() {
		defer wg.Done()
		mu2.Lock()
		mu1.Lock()
		mu1.Unlock()
		mu2.Unlock()
	}()

	wg.Wait()
}
