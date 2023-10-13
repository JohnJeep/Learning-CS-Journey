# Actor Model

Actor模型是一个概念模型，用于处理并发计算。Actor由3部分组成：状态（State）+行为（Behavior）+邮箱（Mailbox），State是指actor对象的变量信息，存在于actor之中，actor之间不共享内存数据，actor只会在接收到消息后，调用自己的方法改变自己的state，从而避免并发条件下的死锁等问题；Behavior是指actor的计算行为逻辑；邮箱建立actor之间的联系，一个actor发送消息后，接收消息的actor将消息放入邮箱中等待处理，邮箱内部通过队列实现，消息传递通过异步方式进行。



## 带着问题思考

1. Actors是如何发送和接收消息的？

   在Actor模型中，消息是唯一的通信机制。Actors通过消息进行相互通信，它们不直接共享内存。以下是Actors如何发送和接收消息的基本概念：

   ### 1. **发送消息：**

   Actors可以发送消息给其他Actors，也可以发送消息给自己。当一个Actor想要与另一个Actor通信时，它创建一个消息并将其发送给目标Actor的地址（通常称为PID，即Process ID）。

   在发送消息时，Actor系统负责将消息传递给正确的接收者。发送消息通常是异步的，这意味着发送者Actor不会等待接收者Actor处理消息。发送消息的过程通常是非阻塞的，这样可以提高系统的并发性能。

   ### 2. **接收消息：**

   当一个Actor接收到消息时，它会将消息放入自己的邮箱（Mailbox）中。邮箱是一个消息队列，用于存储Actor接收到的消息。接收者Actor从邮箱中获取消息，并根据消息的内容执行相应的操作。

   接收消息的过程通常是同步的，即当Actor正在处理一个消息时，它会等待该消息处理完成，然后再处理下一个消息。这确保了消息的顺序处理，避免了竞态条件和数据一致性问题。

   ### 3. **消息处理：**

   当Actor接收到消息后，它会调用预定义的消息处理函数（也称为处理器）来处理消息。处理器是Actor定义的一部分，用于指定接收到特定类型消息时应该执行的操作。处理器可以是Actor的方法，这样当Actor接收到特定类型的消息时，它会调用相应的方法来处理消息。

   ### 4. **消息传递的特性：**

   在Actor模型中，消息传递具有以下特性：

   - **封装性（Encapsulation）：** 每个Actor都有自己的状态和行为，其他Actor无法直接访问其状态，只能通过消息传递来与其通信。
   - **并发性（Concurrency）：** 多个Actor可以并发地执行，处理各自的消息，从而实现系统级的并发性。
   - **位置透明性（Location Transparency）：** 发送消息时，发送者无需知道接收者Actor的具体位置，只需知道接收者的PID即可。

2. 它们是如何并发地工作的？

   

3. 理解代码中的设计模式和结构

### 介绍

Proto.Actor-go 是一个用于构建分布式应用程序的框架，它基于 Go 语言开发，旨在简化并发和分布式系统的开发。这个框架使用 Actor 模型作为基础，允许开发者将应用程序拆分成独立的 Actor，并通过消息传递进行通信。

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



## Core Features(概念)

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

### Cluster(集群)

https://proto.actor/docs/cluster/

- clustering 的核心是 `cluster provider`

- virtual actor model 借鉴了 `Microsoft Orleans` 的概念。一个`Grain` 就是一个 `virtual actors`
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
- Microsoft Orleans: https://learn.microsoft.com/en-us/dotnet/orleans/overview
- 知乎深入解析actor 模型（一)： actor 介绍及在游戏行业应用：https://zhuanlan.zhihu.com/p/427806717
- 知乎深入解析actor 模型（二)： actor 在go 实践proto.Actor 源码解析：https://zhuanlan.zhihu.com/p/427817175

