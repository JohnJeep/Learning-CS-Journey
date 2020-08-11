<!--
 * @Author:JohnJeep
 * @Date: 2020-05-28 21:45:05
 * @LastEditTime: 2020-08-11 20:43:33
 * @LastEditors: Please set LastEditors
 * @Description: 信号量(semaphore)
--> 
# semaphore
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
### Deadlock（死锁）
- 消费者和生产者都在相互的等待对方，就发生了死锁的情况。
- 解决方法
  - 减少锁的作用域（scope）。
    > 多线程常用的模式：有界缓冲（bounded buffer）。将互斥锁的获取和释放操作移到临界区附近，将 full 和empty的等待和唤醒操作移动到锁的外面。


### Reader-Writer Locks（读者-写者锁）
- 不同的数据结构可能访问不同类型的锁。
- 某个线程要更新数据结构，需要调用 `rwlock_acquire_lock()` 来获得锁，调用 `rwlock_release_writelock()` 来释放锁。内部通过一个 write 的信号量保证只有一个写着(writer)能获得锁，进入临界状态，从而更新数据结构。
- 特点
  - 写独占，读共享 
  - 写锁优先级高，即进程中有写操作，读操作则会被阻塞。
  - 使用于读的次数远大于写的次数

- 缺点
  - 缺少公平性。会导致 `reader` 很容易饿死。
  - 实现方案很复杂，导致更多的性能开销。


## The Dining Philosophers（哲学家就餐问题）
- 什么是哲学家就餐问题？
  > 5 位哲学家围绕一个圆桌，每位哲学家之间有一把餐叉。哲学家有时需要思考，有时需要餐叉，有时不需要餐叉。而每位哲学家只有同时拿到了左手边和右手边的餐叉，才能吃到东西。
- 这个问题或涉及竞争和同步的问题。


## 怎样实现信号量？
```
typedef struct __Zem_t {
  int value;
  pthread_cond_t cond;
  pthread_mutex_t lock;
} Zem_t;

// only one thread can call this
void Zem_init(Zem_t *s, int value) {
  s->value = value;
  Cond_init(&s->cond);
  Mutex_init(&s->lock);
}

void Zem_wait(Zem_t *s) {
  Mutex_lock(&s->lock);
  while (s->value <= 0)
  Cond_wait(&s->cond, &s->lock);
  s->value--;
  Mutex_unlock(&s->lock);
}

void Zem_post(Zem_t *s) {
  Mutex_lock(&s->lock);
  s->value++;
  Cond_signal(&s->cond);
  Mutex_unlock(&s->lock);
}
```

- `Zemaphores` 信号量实现：只使用了一把锁、一个条件变量、一个状态变量来记录信号量的值。注意：但信号量为负数时，没有考虑它等待的线程数。
- 利用信号量实现锁(lock)和条件变量(condition variables)，是非常棘手的问题。

