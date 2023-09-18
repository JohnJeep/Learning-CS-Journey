# Actor Model



## 特点

- Actor 之间是彼此隔离的，不会共享内存，只能通过邮箱方式去和对方打交道。

  > 邮箱（MailBox）：本质上是一个队列。每个Actor都有一个邮箱，邮箱接收并缓存其他Actor发过来的消息。

- 彼此之间异步发送消息和处理的。

- Actor一次只能**同步**处理一个消息，处理消息过程中，除了可以接收消息外不能做任何其他操作。

- 多个 Actor 之间通过发信息进行交流，actor与actor之间可以交流，但是不会产生数据竞争。

- 一个Actor可以响应消息、退出新Actor、改变内部状态、将消息发送到一个或多个Actor。

- Actor可能会堵塞自己但Actor不应该堵塞自己运行的线程

- 多个actor向同一actor发送消息，按照时间顺序投递进对方MailBox

- actor 能够根据到来的消息通过行为修改自己的状态，可以发送信息给其他actor，可以创建新的actor或者新的子actor。

- actor需要知道接收方的地址，需要知道将消息传递给谁

## 缺点

- Actor 没有拒绝交流的能力。当消息到达时，必须对其进行处理，或者将其放入缓冲区，以便稍后处理。



## Core Features

### state

ctor本身的属性信息，state只能被actor自己操作，不能被其他actor共享和操作，有效的避免加锁和数据竞争

### behavior

Actor的处理逻辑。定义在该时间点对消息作出反应时要采取的行动的功能，例如，如果客户端得到授权，则转发请求，否则拒绝请求。此行为可能会随着时间的推移而改变

### mailbox

- actor 存储消息是 fifo队列，每个actor只有一个信箱，所有发送者都将他们的消息队列排队。

- actor与actor之间发送消息，消息只能发送到邮箱，等待拥有邮箱的actor 去处理，这个过程是异步的。

### PID

- 创建一个 Actor 时，不能直接向另一个 Actor 发送者消息，而是用开发者暴露出消息传递的一个引用（reference），这个引用就叫 `PID`，是 `process id`的简称。
- PID 是一个序列化标识，被使用发送 messages 到 Actor 的 mailbox。

- 通过 PID 就知道了实际 actor instance 位于哪儿以及是怎样通信的。

### children

### supervisor Strategy

- 每个actor都有一个监督者。
- Proto.Actor实现了一种称为“家长监督”的特定形式。actor只能由其他actor创建，其中顶级actor由库提供，每个创建的actor由其父级监控。
- 两类监管策略
  - ForOneStrategy：将获得的指令应用于失败的子级，没有明确指定，这就是默认值。
  - AllForOneStrategy：适用于child actor 之间存在紧密依赖关系的情况，即一个child的失败会影响其他child的功能，他们之间存在着不可分割的联系。

### messages

- Actor 可以运行在本地进程，也可以在不同机器的远程Actor上运行。
- 消息是不可变的，因此是线程安全的。
- 消息是异步的，

### Cluster

https://proto.actor/docs/cluster/

- clustering 的核心是 `cluster provider`

- virtual actor model 借鉴了 `Microsoft Orleans` 的概念。`proto.Actor` 使用 `Grain` 为代码生成 `virtual actors`
- Goissp
- pub-sub











重启期间事件的详细顺序如下所示：

1. 挂起actor，在这之间将不能处理普通消息直到恢复过来
2. 调用实例的`PreRestart`钩子(默认发送终止请求给所有的孩子，同时调用 postStop)
3. 等待所有请求终结的孩子在`PreRestart`之间(使用`context.Stop()`真正的停止; 像所有children actor 终结一样, 上一次终结child 的提示 将会影响下一步的处理
4. 通过原来actor的创建函数创建一个新的actor。
5. 新实例调用 `PostRestart`，默认已经调用了`PreStart`。
6. 发送重启消息给所有在步骤三被杀死的孩子，重启孩子将会递归的遵循同一个 步骤2 的process 。
7. 恢复actor。







# References

- offical website: https://proto.actor/docs/
- [Golang] protoactor-go 101: How actors communicate with each other: https://blog.oklahome.net/2018/09/protoactor-go-messaging-protocol.html
- 知乎深入解析actor 模型（一)： actor 介绍及在游戏行业应用：https://zhuanlan.zhihu.com/p/427806717
- 知乎深入解析actor 模型（二)： actor 在go 实践proto.Actor 源码解析：https://zhuanlan.zhihu.com/p/427817175

