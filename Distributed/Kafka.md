# 1. 简介

**Kafka**是由[Apache软件基金会](https://zh.wikipedia.org/wiki/Apache软件基金会)开发的一个[开源](https://zh.wikipedia.org/wiki/开源)[流处理](https://zh.wikipedia.org/wiki/流处理)平台，由 [Scala](https://zh.wikipedia.org/wiki/Scala) 和 [Java ](https://zh.wikipedia.org/wiki/Java)编写。该项目的目标是为处理实时数据提供一个统一、高吞吐、低延迟的平台。其持久化层本质上是一个“按照分布式事务日志架构的大规模发布/订阅消息队列”，这使它作为企业级基础设施来处理流式数据非常有价值。此外，Kafka 可以通过Kafka Connect 连接到外部系统（用于数据输入/输出），并提供了 Kafka Streams——一个 Java 流式处理库。

Kafka 是一个分布式的基于发布-订阅模式的消息队列（Message Queue，主要应用于大数据实时处理领域。  

# 2. 消息队列

好处

- 解耦
- 削峰

两种模式

1. 点对点模式（一对一）
2. 发布-订阅模式（一对多，消费）
   - 消费者的消息由消费者自己来决定，消费者主动拉取数据。缺点：一直在轮询，看消息队列中是否有数据。
   - 生产者主动 push 数据。

# 3. Kafka 架构





Topic（主题）：将数据分类，主题中有分区有副本。kafka 中的数据主要存储在 topic 中。

Partition（分区）：提高 topic 的负载均衡，同时也提高了并发能力。

Follower：相当于备份的作用。

注意：同一个分区（partition）中的数据（topic）只能被同一个组中的一个消费者（consumer）消费。

Zookeeper 作用

- 帮助 kafka 存储信息
- 存储消费者消费的位置信息。

kafka 消息存储在磁盘。



# 4. 工作流程

*Seek 直接找数据的位置*

保证 partition 有序，不能保证全局有序。



定位 index 采用 二分查找，

offset + dataSzie

# 5. 生产者

## 5.1. 分区策略

## 5.2. 数据可靠性保证

ISR(In-sync replica set)：同步副本；作用：leader 挂了以后去选择一个新的 leader。

- 保留了时间参数：0.9版本之前，超过数目参数的最大值后，频繁的要访问内存和Zookeeper，降低了效率，因此删除了。

acks 参数配置

- 0：
- 1：

ack 解决的是数据丢失和数据重复问题。

HW 解决的是消费一致性和存储一致性问题。



# 6. 消费者