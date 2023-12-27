package main

import (
	"fmt"
	"sync"
	"time"
)

type Worker interface {
	Task()
}

type Pool struct {
	work chan Worker
	wg   sync.WaitGroup
}

// 创建一个工作池
func New(maxNum int) *Pool {
	p := Pool{
		work: make(chan Worker),
	}

	p.wg.Add(maxNum)
	for i := 0; i < maxNum; i++ {
		go func() {
			for w := range p.work {
				w.Task()
			}
			p.wg.Done()
		}()
	}

	return &p
}

// 提交到工作池
func (p *Pool) Run(w Worker) {
	p.work <- w
}

// 等待所有 goroutine 停止工作
func (p *Pool) ShutDown() {
	close(p.work)
	p.wg.Wait()
}

var names = []string{
	"steve",
	"boob",
	"mary",
	"jason",
}

type namePrinter struct {
	name string
}

func (n *namePrinter) Task() {
	fmt.Println("n.name", n.name)
	time.Sleep(time.Second)
}

func main() {
	p := New(2)
}
