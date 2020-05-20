```
 * @Author: your name
 * @Date: 2020-05-18 22:01:10
 * @LastEditTime: 2020-05-18 22:01:11
 * @LastEditors: Please set LastEditors
 * @Description: 线程(thread)的API接口
```
如何构建并发程序的逻辑？

`man -k pthread` 查看所有线程接口的API


### pthread_create
参数
- `thead` 指向`pthread_t`结构体类型的指针
- `attr`  指定线程具有的属性（包括：栈大小、线程调度优先级）
- `(*start_routine)(void*)` 一个函数指针，告诉这个线程应该在哪个函数中运行。
- `arg`  要传递给线程指向的函数中开始执行的参数


### pthread_join: 等待线程完成
参数
- `__th` 指定要等待的线程
- `**__thread_return` 需要传入一个指向**传入参数值**的指针，而不是传入参数值的本身。

使用`join`确保在退出或进入下一阶段计算之前完成所有线程的工作。



### 锁(lock)
- `pthread_mutex_lock`
- `pthread_mutex_unlock`


- 两种初始化锁方法
  - `pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;`  这样做锁会这只默认值
  - 在运行时设置锁 `int rc = pthread_mutex_init(&lock, NULL);`
 - 锁用完后调用 `pthread mutex destroy()` 


- 在上锁或解锁的时候，常常需要检查错误代码(error codes)，即加断言(assert)。防止程序在存现问题时，不会简单的退出。


### 条件变量(Condition Variables)
- `int pthread_cond_wait(pthread_cond_t *cond, pthread_mutex_t *mutex);`
- `int pthread_cond_signal(pthread_cond_t *cond);`
- `pthread cond wait()` 调用线程进入休眠，让调用者在睡眠时释放锁。再被唤醒之后返回之前，`pthread cond wait()`会重新获得锁，从而确保等待线程在等待序列开始时获取与结束时释放锁之间运行的任何时间，它都有锁。


### 使用 POSIX thread library遵循的准则
- 简单化(Keep it simple): 线程之间的锁和信号代码应该尽可能简单，复杂的线程交互容易产生 bug。
- 最小化线程交互(Minimize thread interactions): 每次交互都应该仔细的想清楚，用验证过的、正确的方法来实现。
- 初始化锁和条件变量(Initialize locks and condition variables): 未初始化的代码有时正常，有时错误，可能或产生奇怪的结果。
- 检查返回值(Check your return codes)
- 注意传给线程的参数和返回值(Be careful with how you pass arguments to, and return values from, threads)
- 每个线程都有自己的栈( Each thread has its own stack)：每个线程之间的局部变量是私有的，共享数据必须在 `heap`上或者在其它全局可以访问的位置上。
- 线程之间总是通过条件变量发送信号(Always use condition variables to signal between threads): 不要使用标志位(flag)来同步。
- 使用参考手册(Use the manual pages)