```
 * @Author: your name
 * @Date: 2020-05-25 23:01:38
 * @LastEditTime: 2020-05-25 23:01:39
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
``` 
### 锁的并发数据结构
- 并发计数器（Concurrent Counters）
- 并发链表（Concurrent Linked Lists）
- 并发队列（Concurrent Queues）
- 并发散列表（Concurrent Hash Table）


### 条件变量（Condition Variables）
- `join()` 父线程检查子线程是否实行完毕。
- 条件变量有两种相关操作
  - `wait()`    线程希望自己睡眠的时候，调用
  - `signal()`  当线程更改了程序中的某些内容和希望唤醒处于睡眠状态的线程时调用
  - 注意：在调用 `wait()` 和 `signal()`时，线程要持有锁，否则会产生一些意想不到的错误。

### 生产者和消费者（有界缓冲问题）（The Producer/Consumer (Bounded Buffer) Problem）
- 多线程在检查条件变量（condition variables）时，应使用 `while`，而少使用 `if` 进行循环。
- 使用两个条件变量替代一个条件变量来正确的发信号，指示在系统状态改变时，哪类线程应该被唤醒。
- 在采用生产者和消费者模式时应注意：保证消费者不应该唤醒消费者，生产者不应该唤醒生产者。
- 要保证高并发和效率：需增加多个缓冲位，保证在睡眠之前，多个值被消费（consumed），同样的在睡眠之前，多个值被生产（produced）。
