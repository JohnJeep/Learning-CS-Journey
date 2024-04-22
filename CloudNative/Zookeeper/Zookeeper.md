<!-- TOC -->

- [1. 概念](#1-概念)
- [2. ZooKeeper 应用场景](#2-zookeeper-应用场景)
- [3. 缺点](#3-缺点)
- [4. 特点](#4-特点)
- [5. 数据模型（Data Model）](#5-数据模型data-model)
- [6. data tree 接口](#6-data-tree-接口)
- [7. Znode 分类](#7-znode-分类)
- [8. 事务 ID](#8-事务-id)
- [9. Zookeeper 的角色](#9-zookeeper-的角色)
- [10. 基本命令](#10-基本命令)
- [11. 监听（Watch）](#11-监听watch)
- [12. 面试](#12-面试)
  - [12.1. 选举机制](#121-选举机制)
  - [12.2. 生产集群安装多少 zk 合适？](#122-生产集群安装多少-zk-合适)
- [13. 参考](#13-参考)

<!-- /TOC -->


# 1. 概念

ZooKeeper 是一个分布式的，开放源码的分布式应用程序协同服务。ZooKeeper 的设计目标是将那些复杂且容易出错的分布式一致性服务封装起来，构成一个高效可靠的原语集，并以一系列简单易用的接口提供给用户使用。

Zookeeper=文件系统+监控通知机制

# 2. ZooKeeper 应用场景

很多分布式协调服务都可以用 ZooKeeper 来做，其中典型应用场景如下：

- 配置管理（configuration management）：如果我们做普通的 Java 应用，一般配置项就是一个本地的配置文件，如果是微服务系统，各个独立服务都要使用集中化的配置管理，这个时候就需要 ZooKeeper。
- DNS 服务
- 组成员管理（group membership）：比如上面讲到的 HBase 其实就是用来做集群的组成员管理。
- 各种分布式锁

ZooKeeper 适用于存储和协同相关的关键数据，不适合用于大数据量存储。如果要存 KV 或者大量的业务数据，还是要用数据库或者其他 NoSql 来做。

# 3. 缺点

为什么 ZooKeeper 不适合大数据量存储呢？主要有以下两个原因：

1. 设计方面：ZooKeeper 需要把所有的数据（它的 data tree）加载到内存中。这就决定了ZooKeeper 存储的数据量受内存的限制。这一点 ZooKeeper 和 Redis 比较像。一般的数据库系统例如 MySQL（使用 InnoDB 存储引擎的话）可以存储大于内存的数据，这是因为 InnoDB 是基于 B-Tree 的存储引擎。B-tree 存储引擎和 LSM 存储引擎都可以存储大于内存的数据量。
2. 工程方面：ZooKeeper 的设计目标是为协同服务提供数据存储，数据的高可用性和性能是最重要的系统指标，处理大数量不是 ZooKeeper 的首要目标。因此，ZooKeeper 不会对大数量存储做太多工程上的优化。

# 4. 特点

1）Zookeeper：一个领导者（ Leader） ， 多个跟随者（ Follower） 组成的集群。
2） **集群中只要有半数以上节点存活， Zookeeper集群就能正常服务。 所以Zookeeper适合安装奇数台服务器。**
3） 全局数据一致：每个Server保存一份相同的数据副本， Client无论连接到哪个Server， 数据都是一致的。
4） 更新请求顺序执行， 来自同一个Client的更新请求按其发送顺序依次执行。
5） 数据更新原子性， 一次数据更新要么成功， 要么失败。
6） 实时性， 在一定时间范围内， Client能读到最新数据。  

# 5. 数据模型（Data Model）

![img](https://zookeeper.readthedocs.io/zh/latest/_images/zkservice.jpg)

ZooKeeper 数据模型的结构与 Unix 文件系统很类似，整体上可以看作是一棵树，每个节点称做一个 ZNode。每一个 ZNode 默认能够存储 **1MB** 的数据，每个 ZNode 都可以通过其路径唯一标识。  

ZooKeeper 的数据模型是层次模型。层次模型常见于文件系统。层次模型和 key-value 模型是两种主流的数据模型。ZooKeeper 使用文件系统模型主要基于以下两点考虑：

1. 文件系统的树形结构便于表达数据之间的层次关系。
2. 文件系统的树形结构便于为不同的应用分配独立的命名空间（namespace）。

ZooKeeper 的层次模型称作 data tree。Data tree 的每个节点叫做 znode。不同于文件系统，每个节点都可以保存数据。每个节点都有一个版本(version)，版本从 0 开始计数。

# 6. data tree 接口

ZooKeeper 对外提供一个用来访问 data tree的简化文件系统 API：

- 使用 UNIX 风格的路径名来定位 znode,例如 /A/X 表示 znode A 的子节点 X。
- znode 的数据只支持全量写入和读取，没有像通用文件系统那样支持部分写入和读取。
- data tree 的所有 API 都是 wait-free 的，正在执行中的 API 调用不会影响其他 API 的完成。
- data tree 的 API都是对文件系统的 wait-free 操作，不直接提供锁这样的分布式协同机制。但是 data tree 的 API 非常强大，可以用来实现多种分布式协同机制。

# 7. Znode 分类

一个 znode 可以是持久性的，也可以是临时性的，znode 节点也可以是顺序性的。每一个顺序性的 znode 关联一个唯一的单调递增整数，因此 ZooKeeper 主要有以下 4 种 znode：

1. 持久性的 znode (PERSISTENT): ZooKeeper 宕机，或者 client 宕机，这个 znode 一旦创建就不会丢失。
2. 临时性的 znode (EPHEMERAL): ZooKeeper 宕机了，或者 client 在指定的 timeout 时间内没有连接 server，都会被认为丢失。
3. 持久顺序性的 znode (PERSISTENT_SEQUENTIAL): znode 除了具备持久性 znode 的特点之外，znode 的名字具备顺序性。
4. 临时顺序性的 znode (EPHEMERAL_SEQUENTIAL): znode 除了具备临时性 znode 的特点之外，znode 的名字具备顺序性。

# 8. 事务 ID

为了保证事务的顺序一致性，Zookeeper 采用了递增的事务 ID 号(zxid)来标识事务，所有的操作(proposal)都会在被提出时加上 zxid，zxid 是一个 64 位的数字，他高 32 位是 epoch 用来标识 leader 关系是否发生变化，每当有新的 leader 被选举出来，都会有一个新的 epoch，标识当前属于哪个 leader 的领导。

对于 Zookeeper 来说，每次的变化都会产生一个唯一的事务 id，zxid（ZooKeeper Transaction Id）通过 zxid ，可以确定更新操作的先后顺序，如果说 zxid1 小于 zxid2，说明 zxid1 比 zxid 先发生。



1. SID： 服务器ID。 用来唯一标识一台ZooKeeper集群中的机器，每台机器不能重复， 和myid一致。  
2. ZXID：事务ID。 ZXID是一个事务ID，用来标识一次服务器状态的变更。 在某一时刻，集群中的每台机器的ZXID值不一定完全一致，这和ZooKeeper服务器对于客户端“更新请求”的处理逻辑有关。  
3. Epoch： 每个Leader任期的代号。没有Leader时同一轮投票过程中的逻辑时钟值是相同的。每投完一次票这个数据就会增加  

# 9. Zookeeper 的角色

- **领导者（leader）** ：负责进行投票的发起和决议，更新系统状态
- **学习者（learner）** ：包括跟随者（follower）和观察者（observer），follower 用于接受客户端请求并想客户端返回结果，在选主过程中参与投票
- **Observer** ：可以接受客户端连接，将写请求转发给 leader，但 observer 不参加投票过程，只同步 leader 的状态，observer 的目的是为了扩展系统，提高读取速度
- **客户端（client）** ：请求发起方

# 10. 基本命令

- **create :** 在树中的某个位置创建一个节点
- **delete :** 删除一个节点存在：测试节点是否存在于某个位置
- **deleteall**：删除所有节点
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



# 11. 监听（Watch）

监听原理

1） 首先要有一个main()线程
2） 在main线程中创建Zookeeper客户端， 这时就会创建两个线程， 一个负责网络连接通信（connet） ， 一个负责监听（listener） 。
3） 通过connect线程将注册的监听事件发送给Zookeeper。
4） 在Zookeeper的注册监听器列表中将注册的监听事件添加到列表中。
5） Zookeeper监听到有数据或路径变化， 就会将这个消息发送给listener线程。
6） listener线程内部调用了process()方法。  

![image-20211121214454971](figures/zk-client-server.png)

常见监听点

1. 监听节点上值得变化情况

```sh
// 一台机器上改变节点
[zk: 192.168.0.191(CONNECTED) 10] set /Yongheng/Dan "Yao"

// 另一台机器上监听
[zk: localhost:2181(CONNECTED) 9] get -w /Yongheng/Dan
WATCHER::

WatchedEvent state:SyncConnected type:NodeDataChanged path:/Yongheng/Dan
```

2. 监听节点的子节点变化情况（增加、删除节点）

```sh
// 192.168.0.191 机器上增加节点
[zk: 192.168.0.191(CONNECTED) 12] create /Yongheng/Qi "GuiFu"
Created /Yongheng/Qi

// localhost 机器上监听变化
[zk: localhost:2181(CONNECTED) 1] ls -w /Yongheng
[Dan, Lin, Xue, Yao]
[zk: localhost:2181(CONNECTED) 2]
WATCHER::

WatchedEvent state:SyncConnected type:NodeChildrenChanged path:/Yongheng
```

<font color=red>注意：</font> 无论是节点中的数据还是节点的变化，注册一次，只能监听一次。想再次监听，需要再次注册。  



# Ensemble 

Zookeeper 中的集群不叫 cluster，而是叫 ensemble。

Zookeeper 使用的是一致性协议（consensus protocol），所以推荐每个 ensemble 里应该包含奇数（odd）个节点（比如 3 个、 5 个等），因为只有当 ensemble 里的大多数节点处于可用状态， Zookeeper 才能处理外部的请求。也就是说，如果有一个包含 3 个节点的 ensemble，那么它允许一个节点失效。如果 ensemble 包含 5 个节点，那么它允许 2 个节点失效。  

## 集群中节点个数的选择

假设有一个包含 5 个节点的集群（ensemble），为了将修改的配置（包括交换节点）文件写入到集群，你需要重启每一个节点。如果你的集群无法容忍多个节点失效，那么在进行集群维护时就会存在风险。不过，也不建议一个集群包含超过 7 个节点，因为 Zookeeper 使用了一致性协议，节点过多会降低整个集群的性能。  

## 集群配置

为了将 zookeeper 的服务器配置成集群（ensemble），需要一个公共的配置，列出所有的服务器。每台服务器在数据目录（data directory）中创建一个 myid 文件，用 于指明自己的 ID。  如果集群里服务器的 hostnames  是 `zoo!.example.com, zoo2.example.com, zoo3 .example.com` ，那么配置文件可能是下面这样的：  

```
tickTime=2000
dataDir=/var/lib/zookeeper
clientPort=2181
initLimit=20
syncLimit=5
server.1=zoo1.example.com:2888:3888
server.2=zoo2.example.com:2888:3888
server.3=zoo3.example.com:2888:3888
```

- clientPort：客户端端口号
- initLimit：表示 followers  连接到 leader 之间建立初始化连接的时间上限。
- syncLimit：表示允许从节点（followers）与主节点（leader）处于不同步状态的时间上限。  
  
  > initLimit 和 syncLimit 单位时间是 tickTime。比如：initLimit 的值为20，表示的时间为：20*2000ms=40s

配置里还列出了集群中所有服务器的地址，服务器地址遵循的格式 `server.X=hostname:peerPort:leaderPort`各个参数说明如下：  

- X：服务的 ID 号，必须是一个整数（integer），不需要从 0 开始或不要求是连续的。
- hostname：服务器的主机名（hostname）或 IP 地址。
- peerPort：集群中的服务器彼此之间通信的端口号。
- leaderPort：leader 选择执行的 TCP 端口号。

客户端只需要通过 clientPort 就能连接到集群，而集群节点间的通信则需要同时用到这 3 个
端口（ peerPort 、 leaderPort 、 clientPort ）。  

除了公共的配置文件外，每个服务器都必须在 `data Dir` 目录中创建一个叫作 `myid` 的文件，文件里要包含服务器 ID ， 这个 ID 要与配置文件里配置的 ID 保持一致。完成这些步骤后，就可以启动服务器，让它们彼此间进行通信了。  

# 12. 面试

## 12.1. 选举机制

半数机制，超过半数的投票通过，即通过。
（1）第一次启动选举规则：投票过半数时， 服务器 id 大的胜出
（2）第二次启动选举规则：
  - EPOCH 大的直接胜出
  - EPOCH 相同，事务 id 大的胜出
  - 事务 id 相同，服务器 id 大的胜出  


## 12.2. 生产集群安装多少 zk 合适？
安装奇数台。
生产经验：

- 10 台服务器： 3 台 zk；
- 20 台服务器： 5 台 zk；
- 100 台服务器： 11 台 zk；
-  200 台服务器： 11 台 zk

服务器台数多：好处，提高可靠性；坏处：提高通信延时  

# 13. References

- [Zookeeper 入门介绍](https://xie.infoq.cn/article/edd698410f30cf4b8113e228d)
- [Centos7 离线安装 zookeeper 并设置服务开机自启 实践笔记](https://cloud.tencent.com/developer/article/1913682)