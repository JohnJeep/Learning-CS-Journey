```
 * @Author: JohnJeep
 * @Date: 2020-05-20 22:25:04
 * @LastEditTime: 2020-05-20 22:25:05
 * @LastEditors: Please set LastEditors
 * @Description: 线程锁问题
```
### 什么是锁(lock)?
- 是一个某种类型的变量。
- 这个锁变量(简称锁)保持了锁在某一刻的状态。要么是可用的(available or or unlocked or free)，表示当前没有线程持有锁，线程可以使用该锁；要么是被占用的(acquired or locked or held)，表示当前有线程在临界区( critical section)使用锁。
- 锁的持有者(the owner of the lock): Calling the routine lock() tries to acquire the lock; if no other thread holds the lock (i.e., it is free), the thread will acquire the lock and enter the critical section;
- 锁是为编程者提供了最小程度的调度控制(control over scheduling)。
- 什么是互斥量(mutual exclusion)？
  - POSIX 库将锁称为互斥量。用来提供线程之间的互斥。
- 使用不同的锁去保护不同的数据和结构，允许更多的线程进入临界区。


### 如何实现一种高效的锁？
- 提供互斥(mutual exclusion)。阻止多线程进入临界区。
- 公平性(fairness)。是否每一个竞争(contend)线程都有公平的机会抢到锁。
- 性能(performance)。使用锁增加的时间开销。


### 控制中断(Controlling Interrupts)
- 任何一个调用的线程需要执行一个特权的指令去打开和关闭中断，并且信任线程不会滥用资源。
- 不能再多处理器上实现。
- 长时间关闭中断会导致中断丢失，导致系统会出现一些问题。
- 与普通指令执行相比，屏蔽或取消屏蔽中断的代码(code that masks or unmasks
interrupts)往往会被现代CPU缓慢执行。