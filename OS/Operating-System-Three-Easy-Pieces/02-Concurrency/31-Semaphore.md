<!--
 * @Author: your name
 * @Date: 2020-05-28 21:45:05
 * @LastEditTime: 2020-05-28 23:29:25
 * @LastEditors: Please set LastEditors
 * @Description: 信号量(semaphore)
--> 
## 问题（crux）
- 怎样使用 semaphores 替代 locks 和 condition variables?
- 什么是 semaphores？
- 什么是 binary semaphore(二值信号量)？
- 用锁和条件变量来实现信号量是否简单？
- 不用锁和条件变量怎样来实现信号量？


## 信号量（semaphore）
- 定义：信号量是一个整型值的对象，用两个程序（routines）操作它。
  - `sem_wait()`
  - `sem_post()` 


- binary semaphore
  - locked: 信号量的值设置为 1
  - unlocked: 信号量的值设置为 0


- Semaphores For Ordering（信号量的顺序）
  - 在子进程调用 `sem_post` 之前，父进程首先会调用 `sem_wait` 
  - 在父进程会调用 `sem_wait` 之前，子进程会首先调用 `sem_wait()` 


## The Producer/Consumer (Bounded Buffer) Problem


###  Deadlock（死锁）
- 消费者和生产者都在相互的等待对方，就发生了死锁的情况。

- 解决方法
  - 减少锁的作用域（scope）。
    > 多线程常用的模式：有界缓冲（bounded buffer）。将互斥锁的获取和释放操作移到临界区附近，将 full 和empty的等待和唤醒操作移动到锁的外面。

### Reader-Writer Locks（读者-写者锁）
不同的数据结构可能访问不同类型的锁。












