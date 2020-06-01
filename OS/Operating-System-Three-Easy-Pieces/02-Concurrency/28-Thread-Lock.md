<!--
 * @Author: your name
 * @Date: 2020-05-20 22:25:04
 * @LastEditTime: 2020-06-01 13:39:30
 * @LastEditors: Please set LastEditors
 * @Description: 线程锁问题
--> 

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
- 不能在多处理器上实现。
- 长时间关闭中断会导致中断丢失，导致系统会出现一些问题。
- 与普通指令执行相比，屏蔽或取消屏蔽中断的代码(code that masks or unmasks
interrupts)往往会被现代CPU缓慢执行。


### 自旋锁(Spin Locks)
- test-and-set instruction也叫原子交换（atomic exchange）。
- 互斥锁怎样工作
  > 通过测试旧锁的值和设置新锁的值，进行单个原子操作 ，确保只有一个线程获得该锁。
- 抢占式调度（preemptive scheduler）：不断通过时钟去中断一个线程，让其它的线程可以运算。

```
自旋锁是一种最简单的锁，在循环里面一直等待，利用CPU，直到有锁可用为止。
void lock(lock_t *lock) {
  while (TestAndSet(&lock->flag, 1) == 1)
      ; // spin-wait (do nothing)
}
```

- 评价自旋锁（Evaluating Spin Locks）
  - 正确性（correctness）: 自旋锁仅仅允许单个线程在一定的时间内进入临界区（critical section）
  - 公平性（fairness）: 没有提供让任意一个线程进入都可以进入临界区的保证，没有公平性，可能会导致有线程不会进入临界区（即，饿死）。
  - 性能（performance）
    - 单CPU：对性能的开销很大。当一个线程持有锁进入临界区时被抢占（preempted），调度器可能会运行其它的线程，其它线程都在竞争（contending）锁。在放弃CPU的使用权时，每个线程都会自旋（spin 即，等待）一个时间片段（preempted），导致浪费了CPU的周期。
    - 多CPU：效果很好。自旋（spin）等待其它处理器上的锁是否可用，不会浪费CPU太多的周期时间。


- 比较和交换（Compare-And-Swap）
  - 在 SPARC 系统中，使用的是 `compare-and-swap instruction`
  - 在 x86 系统时，使用的是 `compare-and-exchange instruction`
  - Compare-And-Swap指令比 test-and-set指令更加强大。


### 存储条件指令和加载链接指令（Load-Linked and Store-Conditional）
- Load-Linked: 从内存中取出值存到寄存器中。
- Store-Conditional: 只有在没有发生中间存储到相应地址的情况下，Store-Conditional才会成功（并更新存储在Load-Linked地址上的值）。成功时，返回值为1，并将ptr指向的值更新为value；失败时，返回值为0，不会更新值。


### 获得和增加指令（Fetch-And-Add）
- 原子性的增加一个值，并在特定的地址返回一个旧的值
- 采用 `Fetch-And-Add` 指令能够保证所有的线程都能抢到锁
```
int FetchAndAdd(int *ptr) {
    int old = *ptr;
    *ptr = old + 1;
    return old;
}
```

### 如何避免锁在CPU上浪费过多的自旋等待(spin)？
不仅需要硬件的支持，还需要 OS 的支持

- 当前线程在等待（spin）的时候，让出或放弃CPU（just yield）
  - 线程调用 `yield()` 函数时，线程会主动放弃 CPU，让其它的线程运行。即当前线程的状态由运行态（running）变为就绪态（ready），从而使其它的线程运行。
  - 在单CPU上使用 `yield()` 运行多线程的方式很有效。当一个线程调用 `lock()` 时，发现锁被占用，让出CPU，让另外的线程运行，并完成临界区。
  
- 缺点
  - 这种方法不能用在多个线程反复竞争（contending）一把锁的情况。因为没有解决线程或饿死的问题，一个线程可能会无限的处在 `让出CPU的循环`，而其它的线程会反复地进入和推出临界区。
  - 可能会导致优先级翻转问题（priority inversion）。在现代操作系统中为了克服这一问题，使用优先级继承 `priority inheritance`，让所有的线程都有一样的优先级。


### 使用队列：睡眠替代一直等待(Using Queues: Sleeping Instead Of Spinning)
- 使用队列的目的：处理等待的线程，谁在接下来可以获得锁。
- 在Solaris系统中共采用 `park()` 函数：让调用的线程休眠
- 在Solaris系统中共采用 `unpack()` 函数：通过 `threadID` 唤醒特定的线程。使用 `unpack()` 和 `pack()` 这两个函数，让调用者在获取不到线程时休眠（sleep），在有锁可用时被唤醒（wake）。
- 在Solaris系统中共采用 `setpark()` 解决唤醒/等待竞争（wakeup/waiting race）问题。



- Linux操作系统支持 
  - 提供了 `futex` 接口，让每个 `futex`都关联一个特定的物理内存位置。
  - `futex wait(address, expected)` 当 `address`中的值等于 `expected` 中的值时，让被调用的线程休眠（sleep），如果两者不相等，则调用立刻返回。
  - `futex wake(address)` 唤醒一个在队列中等待的线程。
  - 代码存储在 gnu libc的 `nptl` 库中。
   > 核心思想：利用一个整数，同时记录锁是否被持有（整数的最高位）和线程等待者的个数（整数的其余部位）。若锁是负的，表示该锁被持有（held）。


- 两相锁（Two-Phase Locks）
  - 采用两个阶段处理锁。
  - 第一个spin阶段：先自己等待（自旋：spin）一段时间，希望可以获得锁。
  - 第一个spin阶段：若在第一个阶段没有获得锁，则第二阶段的调用者会进入睡眠（sleep），直到锁可以用。


- Linux系统采用的就是两阶锁，但是它 `自旋的时间` 仅仅只有一次。
- 常见的方式：在使用 `futex` 睡眠之前，在循环中固定 `自旋的时间` 的次数。

