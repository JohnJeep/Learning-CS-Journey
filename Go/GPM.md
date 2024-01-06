# GPM Model

在Golang（也称为Go）中，GPM代表着Go中的并发模型，它包括三个关键的组件：Goroutines（协程）、Scheduler（调度器）和系统线程（M：Machine）。GPM模型是Go语言实现并发的核心。下面是关于GPM模型的简要解释：

1. G(Goroutine)
   - Goroutine是Go语言中的轻量级线程，它由Go运行时（runtime）管理。与传统的线程相比，Goroutines的创建和销毁更加高效。
   - Goroutines通过`go`关键字启动，可以在程序中创建数千甚至数百万个Goroutines而不会消耗太多内存。
   - 每个Goroutine都运行一个函数，它们可以异步执行，互不干扰。
   - 存储了 goroutine 执行的栈信息、goroutine 状态及goroutine的任务函数。
   
2. P(Processor)：协程执行需要的上下文。
   
   - P 的数量决定了系统内最大可并行的G数量。P 中拥有的是各种G对象队列、链表、一些缓存和状态。
   
   - Go运行时包含了一个调度器，它负责管理和调度Goroutines的执行。
   - 调度器会在多个系统线程（M）之间分配Goroutines的执行。
   - 调度器使用一种叫做"work-stealing"的技术，来确保各个系统线程都有足够的工作负载。
   
3. M(Machine)：操作系统的主线程，也叫工作线程。
   - 系统线程是Go运行时的底层执行单元，它们由操作系统管理。
   - 一个Go程序可以使用多个系统线程，每个线程都可以同时运行一个Goroutine。
   - 系统线程会被调度器用来执行Goroutines。

GPM模型的工作流程如下：

1. 当一个Go程序启动时，它通常会创建一个或多个系统线程（M），这些线程称为"工作线程"。
2. 当你使用`go`关键字启动一个新的Goroutine时，调度器会将这个Goroutine分配给一个工作线程来执行。
3. 如果一个工作线程的Goroutine执行完毕或者发生了阻塞（例如等待I/O操作完成），那么调度器会将另一个Goroutine分配给该线程，以充分利用线程的资源。
4. 如果某个Goroutine发生了死锁或者其他无法继续执行的情况，Go运行时会触发恢复机制（panic recovery）来防止整个程序崩溃。
5. 调度器会周期性地检查Goroutines的状态，以确保它们都能够正常执行。

总之，GPM模型是Go语言并发性能的核心之一，它使得Go程序能够高效地运行大量的并发任务而无需过多的线程管理开销。这种模型的好处包括更高的并发性能、更低的内存开销以及更简单的并发编程模型。

## References
- https://golang.design/under-the-hood/zh-cn/part2runtime/ch06sched/mpg/