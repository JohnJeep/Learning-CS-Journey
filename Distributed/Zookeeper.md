# 概念

ZooKeeper 是一个分布式的，开放源码的分布式应用程序协同服务。ZooKeeper 的设计目标是将那些复杂且容易出错的分布式一致性服务封装起来，构成一个高效可靠的原语集，并以一系列简单易用的接口提供给用户使用。

Zookeeper=文件系统+监控通知机制

# ZooKeeper 应用场景

很多分布式协调服务都可以用 ZooKeeper 来做，其中典型应用场景如下：

- 配置管理（configuration management）：如果我们做普通的 Java 应用，一般配置项就是一个本地的配置文件，如果是微服务系统，各个独立服务都要使用集中化的配置管理，这个时候就需要 ZooKeeper。
- DNS 服务
- 组成员管理（group membership）：比如上面讲到的 HBase 其实就是用来做集群的组成员管理。
- 各种分布式锁

ZooKeeper 适用于存储和协同相关的关键数据，不适合用于大数据量存储。如果要存 KV 或者大量的业务数据，还是要用数据库或者其他 NoSql 来做。

为什么 ZooKeeper 不适合大数据量存储呢？主要有以下两个原因：

1. 设计方面：ZooKeeper 需要把所有的数据（它的 data tree）加载到内存中。这就决定了ZooKeeper 存储的数据量受内存的限制。这一点 ZooKeeper 和 Redis 比较像。一般的数据库系统例如 MySQL（使用 InnoDB 存储引擎的话）可以存储大于内存的数据，这是因为 InnoDB 是基于 B-Tree 的存储引擎。B-tree 存储引擎和 LSM 存储引擎都可以存储大于内存的数据量。
2. 工程方面：ZooKeeper 的设计目标是为协同服务提供数据存储，数据的高可用性和性能是最重要的系统指标，处理大数量不是 ZooKeeper 的首要目标。因此，ZooKeeper 不会对大数量存储做太多工程上的优化。

# 特点

1）Zookeeper：一个领导者（ Leader） ， 多个跟随者（ Follower） 组成的集群。
2） **集群中只要有半数以上节点存活， Zookeeper集群就能正常服务。 所以Zookeeper适合安装奇数台服务器。**
3） 全局数据一致：每个Server保存一份相同的数据副本， Client无论连接到哪个Server， 数据都是一致的。
4） 更新请求顺序执行， 来自同一个Client的更新请求按其发送顺序依次执行。
5） 数据更新原子性， 一次数据更新要么成功， 要么失败。
6） 实时性， 在一定时间范围内， Client能读到最新数据。  

# 数据模型（Data Model）

![img](https://zookeeper.readthedocs.io/zh/latest/_images/zkservice.jpg)

ZooKeeper 数据模型的结构与 Unix 文件系统很类似，整体上可以看作是一棵树，每个节点称做一个 ZNode。每一个 ZNode 默认能够存储 **1MB** 的数据，每个 ZNode 都可以通过其路径唯一标识。  

ZooKeeper 的数据模型是层次模型。层次模型常见于文件系统。层次模型和 key-value 模型是两种主流的数据模型。ZooKeeper 使用文件系统模型主要基于以下两点考虑：

1. 文件系统的树形结构便于表达数据之间的层次关系。
2. 文件系统的树形结构便于为不同的应用分配独立的命名空间（namespace）。

ZooKeeper 的层次模型称作 data tree。Data tree 的每个节点叫做 znode。不同于文件系统，每个节点都可以保存数据。每个节点都有一个版本(version)，版本从 0 开始计数。

# data tree 接口

ZooKeeper 对外提供一个用来访问 data tree的简化文件系统 API：

- 使用 UNIX 风格的路径名来定位 znode,例如 /A/X 表示 znode A 的子节点 X。
- znode 的数据只支持全量写入和读取，没有像通用文件系统那样支持部分写入和读取。
- data tree 的所有 API 都是 wait-free 的，正在执行中的 API 调用不会影响其他 API 的完成。
- data tree 的 API都是对文件系统的 wait-free 操作，不直接提供锁这样的分布式协同机制。但是 data tree 的 API 非常强大，可以用来实现多种分布式协同机制。

# Znode 分类

一个 znode 可以是持久性的，也可以是临时性的，znode 节点也可以是顺序性的。每一个顺序性的 znode 关联一个唯一的单调递增整数，因此 ZooKeeper 主要有以下 4 种 znode：

1. 持久性的 znode (PERSISTENT): ZooKeeper 宕机，或者 client 宕机，这个 znode 一旦创建就不会丢失。
2. 临时性的 znode (EPHEMERAL): ZooKeeper 宕机了，或者 client 在指定的 timeout 时间内没有连接 server，都会被认为丢失。
3. 持久顺序性的 znode (PERSISTENT_SEQUENTIAL): znode 除了具备持久性 znode 的特点之外，znode 的名字具备顺序性。
4. 临时顺序性的 znode (EPHEMERAL_SEQUENTIAL): znode 除了具备临时性 znode 的特点之外，znode 的名字具备顺序性。

# 事务 ID

为了保证事务的顺序一致性，Zookeeper 采用了递增的事务 ID 号(zxid)来标识事务，所有的操作(proposal)都会在被提出时加上 zxid，zxid 是一个 64 位的数字，他高 32 位是 epoch 用来标识 leader 关系是否发生变化，每当有新的 leader 被选举出来，都会有一个新的 epoch，标识当前属于哪个 leader 的领导。

对于 Zookeeper 来说，每次的变化都会产生一个唯一的事务 id，zxid（ZooKeeper Transaction Id）通过 zxid ，可以确定更新操作的先后顺序，如果说 zxid1 小于 zxid2，说明 zxid1 比 zxid 先发生。



1. SID： 服务器ID。 用来唯一标识一台ZooKeeper集群中的机器，每台机器不能重复， 和myid一致。  
2. ZXID：事务ID。 ZXID是一个事务ID，用来标识一次服务器状态的变更。 在某一时刻，集群中的每台机器的ZXID值不一定完全一致，这和ZooKeeper服务器对于客户端“更新请求”的处理逻辑有关。  
3. Epoch： 每个Leader任期的代号。没有Leader时同一轮投票过程中的逻辑时钟值是相同的。每投完一次票这个数据就会增加  

# Zookeeper 的角色

- **领导者（leader）** ：负责进行投票的发起和决议，更新系统状态
- **学习者（learner）** ：包括跟随者（follower）和观察者（observer），follower 用于接受客户端请求并想客户端返回结果，在选主过程中参与投票
- **Observer** ：可以接受客户端连接，将写请求转发给 leader，但 observer 不参加投票过程，只同步 leader 的状态，observer 的目的是为了扩展系统，提高读取速度
- **客户端（client）** ：请求发起方

# 基本命令

- **create :** 在树中的某个位置创建一个节点
- **delete :** 删除一个节点存在：测试节点是否存在于某个位置
- **get data :** 从节点读取数据
- **set data：** 将数据写入节点
- **get children :** 检索节点的子节点列表
- **sync :** 等待数据被传播

1. 查看 Zookeeper 中包含的 key

   ```
   ls /
   ```

   

2. 创建一个新的 Znode 创建成功以后我们可以使用 `ls /`查看我们创建的内容

   ```
   create /zkMxn muxiaonong
   
    ls /
   [zkMxn, zookeeper]
   
   ```

3. 获取创建 Znode 的内容

   ```
   get /zkMxn
   ```

4. 对 zk 所关联的字符串进行设置

   ```
   set /zkMxn mxn666
   ```

5. 删除 Znode

   ```
   delete /zkMxn
   ```









# 参考

[Zookeeper 入门介绍](https://xie.infoq.cn/article/edd698410f30cf4b8113e228d)

