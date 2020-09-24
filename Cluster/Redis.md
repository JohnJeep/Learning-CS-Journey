<!--
 * @Author: JohnJeep
 * @Date: 2020-09-05 23:51:27
 * @LastEditTime: 2020-09-24 16:20:44
 * @LastEditors: Please set LastEditors
 * @Description: redis学习
-->
## 0.1. 参考
- [redis.io](https://redis.io/) : Redis官方网站
<!-- TOC -->

- [0.1. 参考](#01-参考)
- [0.2. 概念](#02-概念)
- [0.3. 数据对象](#03-数据对象)
  - [0.3.1. string(字符串)](#031-string字符串)
  - [0.3.2. hash(哈希)](#032-hash哈希)
  - [0.3.3. list(列表)](#033-list列表)
  - [0.3.4. set(集合)](#034-set集合)
  - [0.3.5. zset(sorted set 有序集合)](#035-zsetsorted-set-有序集合)
- [0.4. 基本命令](#04-基本命令)
  - [0.4.1. KEYS 命令](#041-keys-命令)
  - [0.4.2. EXISTS命令](#042-exists命令)
  - [0.4.3. TYPE 命令](#043-type-命令)
  - [0.4.4. DEL 命令](#044-del-命令)
- [0.5. pub/sub(订阅与发布模式)](#05-pubsub订阅与发布模式)
- [0.6. 事务](#06-事务)
- [0.7. 数据备份与恢复](#07-数据备份与恢复)

<!-- /TOC -->
- [redis.cn-commands](http://redis.cn/commands.html) redis中文版相关命令用法
- [Redis学习教程](https://piaosanlang.gitbooks.io/redis/content/index.html) 比较全面的介绍了有关redis的使用。
- [W3Cschool Redis教程](https://www.w3cschool.cn/redis/redis-intro.html)
- [CSN: Redis教程](https://blog.csdn.net/hellozpc/article/details/81267030)


## 0.2. 概念
- 什么是redis？
  > Redis是一个开源的使用ANSI C语言编写、遵守BSD协议、支持网络、可基于内存亦可持久化的日志型、key-Value 的数据库、并提供多种语言的API。通常，Redis 将数据存储于内存中，或被配置为使用虚拟内存。
  
  > redis是Nosql数据库中使用较为广泛的非关系型内存数据库，redis内部是一个key-value存储系统。它支持存储的value类型相对更多，包括string(字符串)、list(链表)、set(集合)、zset(sorted set –有序集合)和hash（哈希类型，类似于Java中的map）。Redis基于内存运行并支持持久化的NoSQL数据库，是当前最热门的NoSql数据库之一，也被人们称为数据结构存储服务务器。

- Redis的主要优点
  - Redis支持数据的持久化，可以将内存中的数据保持在磁盘中，重启的时候可以再次加载进行使用
  - Redis不仅仅支持简单的 `key-value` 类型的数据，同时还提供 `list`，`set`，`zset`，`hash`等数据结构的存储
  - Redis支持数据的备份，即master-slave模式的数据备份。

## 0.3. 数据对象
### 0.3.1. string(字符串)
- string类型是Redis最基本的数据类型，一个 `key`对应一个 `value` ，一个键最大能存储512MB。

- 通过Redis `STRING` 命令中 `get` 和 `set` 关键字来操作。
    ```
    redis 127.0.0.1:6379> SET name "Jack"
    OK
    redis 127.0.0.1:6379> GET name
    "Jack"
    127.0.0.1:6379> type name
    string
    ```
- 删除键值：`redis 127.0.0.1:6379> del name`，删除这个 key 及对应的 value
- 验证键是否存在
  ```
    redis 127.0.0.1:6379> exists name
    (integer) 0
  ```

### 0.3.2. hash(哈希)
- Redis hash是一个string类型的 `field` 和 `value` 的映射表，是一个键值对集合，hash特别适合用于存储对象。
- Hash中的关键字都在Redis的 `HASHES` 命令中。
    ```
    // 往hash表中插入数据，my_table 是一个键值
    127.0.0.1:6379> HMSET my_table name Anna age 18
    OK
    127.0.0.1:6379> HGETALL my_table
    1) "name"
    2) "Anna"
    3) "age"
    4) "18"
    ```


### 0.3.3. list(列表)
- Redis 列表是简单的字符串列表，按照插入顺序排序。你可以添加一个元素到列表的头部（左边）或者尾部（右边）。
- 列表最多可存储 $2^{32} - 1$ 元素 (4294967295, 每个列表可存储40多亿)。

    ```
    // 使用lpush往list中添加数据，lrange查看list中的数据
    127.0.0.1:6379> lpush my_list apple
    (integer) 1
    127.0.0.1:6379> lpush my_list banana
    (integer) 2
    127.0.0.1:6379> lpush my_list pear
    (integer) 3
    127.0.0.1:6379> lpush my_list peach
    (integer) 4
    127.0.0.1:6379> lrange my_list 0 10
    1) "peach"
    2) "pear"
    3) "banana"
    4) "apple"
    ```


### 0.3.4. set(集合)
- Redis Set是string类型的无序集合。集合是通过哈希表实现的，所以添加，删除，查找的复杂度都是O(1)。 在 Redis 可以添加，删除和测试成员存在的时间复杂度为 O（1）。
- 集合中最大的成员数为 $2^{32} - 1$ (4294967295, 每个集合可存储40多亿个成员)。
-  Redis 非常人性化的为集合提供了 求交集、并集、差集等操作, 那么就可以非常方便的实现如共同关注、共同喜好、二度好友等功能, 对上面的所有集合操作,你还可以使用不同的命令选择将结果返回给客户端还是存集到一个新的集合中。

    ```
    127.0.0.1:6379> sadd my_set one
    (integer) 1
    127.0.0.1:6379> sadd my_set two
    (integer) 1
    127.0.0.1:6379> sadd my_set three
    (integer) 1
    127.0.0.1:6379> smembers my_set
    1) "three"
    2) "two"
    3) "one"
    127.0.0.1:6379> scard my_set
    (integer) 3
    ```

- 命令
  - `sadd my_set member [...]`: 向my_set集合中添加一个或多个member元素
  - `sdiff key1 key2`: 返回ke1集合中除去与key2集合中共有的元素
  - `sinter key1 key2`: 返回key1集合与key2集合的交集的元素
  - `smembers my_set`: 查看my_set集合中所有的元素
  - `scard my_set`: 查看my_set集合中的元素的数量
  - `sismember my_set "one"`: 判断my_set 集合中是否有 one 成员值
  - `smove source destination member`: 将source集合中的member成员移动到destination集合中
  - `spop my_set [count]` 从my_set集合中随机删除元素并返回返回一个或多个随机元素。
    - count参数将在更高版本中提供，但是在2.6、2.8、3.0中不可用。
  - `srandmember my_set [count]` 从my_set集合中随机获取一个元素值
    - 不使用count 参数的情况下该命令返回随机的元素，如果key不存在则返回nil。
    - 使用count参数,则返回一个随机的元素数组，如果key不存在则返回一个空的数组。
      - Redis 2.6开始，可以接受 count 参数，如果count是整数且小于元素的个数，返回含有 count 个不同的元素的数组，
      - 如果count是个整数且大于集合中元素的个数时，仅返回整个集合的所有元素，
      - 当count是负数，则会返回一个包含 count 绝对值的个数元素的数组；如果count的绝对值大于元素的个数，则返回的结果集里会出现一个元素出现多次的情况。
    ```
      127.0.0.1:6379> smembers my_set
      1) "two"
      2) "one"
      3) "three"
      4) "four"
      127.0.0.1:6379> srandmember my_set
      "four"
      127.0.0.1:6379> srandmember my_set
      "one"
      127.0.0.1:6379> srandmember my_set 2
      1) "two"
      2) "three"
      127.0.0.1:6379> srandmember my_set 3
      1) "two"
      2) "one"
      3) "three"
      127.0.0.1:6379> srandmember my_set 5
      1) "two"
      2) "one"
      3) "three"
      4) "four"
      127.0.0.1:6379> srandmember my_set -1
      1) "four"
      127.0.0.1:6379> srandmember my_set -2
      1) "two"
      2) "four"
      127.0.0.1:6379> srandmember my_set -5
      1) "three"
      2) "two"
      3) "two"
      4) "four"
      5) "one"
    ```
    - `srem my_set member [...]` 从my_set集合中删除指定的一个或多个元素。
    - `sunion my_set1 my_set2` 返回my_set1集合与my_set2集合中所有元素的并集。
    - `sunionstore set_key set_key1 set_key2` 返回set_key1集合与set_key2集合中所有元素的并集，并将结果存储在新的集合set_key中。
    - `sscan my_set ` 迭代当前数据库中my_key集合的元素

- 应用场景
  - 1.共同好友、二度好友
  - 2.利用唯一性,可以统计访问网站的所有独立 IP
  - 3.好友推荐的时候,根据 tag 求交集,大于某个 临界值 就可以推荐


### 0.3.5. zset(sorted set 有序集合)
- Redis 有序集合和集合一样也是string类型元素的集合，且不允许重复的成员。redis通过 `score` 来为集合中的成员进行从小到大的排序。有序集合的成员(member)是唯一的,但分数(score)却可以重复。

- 命令
  - `zadd key score member`：将指定成员添加到键值为key的有序集合里面
  - `zcard key`: 返回key集合中的成员数量。
  - `zcount key min max`：返回key集合中分数在最小和最大值之间的成员数据的个数。
  - `zincrby key increment member`:  在有序集合key中的成员数据member的score值加上增量increment值。
  - `zrevrange key start stop [withscores]`: 按照逆序排序返回有序集合key中在start和stop区间范围内的元素。
  - `zrange key start stop [withscores]`: 返回有序集合key中在start和stop区间范围内的元素。start和stop都是全包含的区间。
    - 分数值是一个双精度的浮点型数字字符串。+inf和-inf都是有效值。 
    - 加上 `withscores` 选项时，将元素的分数与元素一起返回。
    ```
    127.0.0.1:6379> zcard my_zset
    (integer) 3
    127.0.0.1:6379> zrange my_zset 0 2 withscores
    1) "first"
    2) "1"
    3) "second"
    4) "2"
    5) "third"
    6) "2"
    127.0.0.1:6379> zcount my_zset 1 3
    (integer) 3
    127.0.0.1:6379> zcount my_zset (1 3
    (integer) 2
    127.0.0.1:6379> zincrby my_zset 5 first
    "6"
    127.0.0.1:6379> zrange my_zset 0 2 withscores
    1) "second"
    2) "2"
    3) "third"
    4) "2"
    5) "first"
    6) "6"
    ```
- 应用场景
  - 1.带有权重的元素,LOL游戏大区最强王者
  - 2 排行榜

## 0.4. 基本命令

### 0.4.1. KEYS 命令
- `key pattern`: 获取符合规则的键名列表
- 示例：`keys *` : 查询数据库中所有的键
- 常用的符号
  - `？`: 匹配一个字符 
  - `*`: 匹配任意个字符（包括0个字符）
  - `[]`: 匹配括号间的任一字符
  - `\*` : 匹配字符x,用于转义符号；若要匹配 ？，则使用 `\?`

### 0.4.2. EXISTS命令
- `exists key`：判断一个按键是否存在，如果按键存在，则返回整数类型1，否则返回0
```
127.0.0.1:6379> exists li
(integer) 0
127.0.0.1:6379> exists wang
(integer) 1
```

### 0.4.3. TYPE 命令
- `del key1, key2 ...`: 删除一个或多个按键，返回值是删除的键的个数。
```
127.0.0.1:6379> del match_rank s_set
(integer) 2
```

### 0.4.4. DEL 命令
- `type key1, key2, ...`: 获取按键的数据类型
  > 数据类型可以是string、hash、list、set、zset。
```
127.0.0.1:6379> keys *
 1) "wang"
 2) "my_set"
 3) "deng"
 4) "my_zset"
 5) "my_table"
 6) "name"
 7) "key2"
 8) "s_set"
 9) "key1"
```


## 0.5. pub/sub(订阅与发布模式)
> Redis 发布订阅(pub/sub)是一种消息通信模式：发送者(publish)发送消息，订阅者(subscribe)接收消息。 Redis 客户端可以订阅任意数量的频道。

- Redis 发布订阅命令
  - `psubscribe pattern [pattern ...]`: 订阅一个或多个符合给定模式的频道，返回接收到的信息。
  - `pubsub subcommand [argument [argument ...]]`: 查看订阅与发布系统状态。
  - `subscribe channel [channel ...]`: 订阅给定的一个或多个频道的信息。
  - `unsubscribe [channel [channel] ...]`: 退订指定的频道。
  - `punsubsrcibe [pattern [pattern] ...]`: 停止发布到指定模式的频道。
  - `publish channel message`: 给频道channel发布一条消息，返回收到消息的客户端数量。 





## 0.6. 事务
- Redis事务允许一组命令在单一步骤中执行。事务具有两个属性
  - 事务是一个单独的隔离操作：事务中的所有命令都会序列化、按顺序地执行。事务在执行的过程中，不会被其他客户端发送来的命令请求所打断。
  - Redis事务是原子的。原子意味着要么所有的命令都执行，要么都不执行；

- 一个事物从开始到执行经历三个阶段：开始事务（multi）、命令入队（queued）、执行事务（exec）
  ```
  // 案例：邓某人给王某人转账500元

  127.0.0.1:6379> set deng 1000
  OK
  127.0.0.1:6379> set wang 200
  OK
  127.0.0.1:6379> multi
  OK
  127.0.0.1:6379> incrby deng -500
  QUEUED
  127.0.0.1:6379> incrby wang 500
  QUEUED
  127.0.0.1:6379> exec
  1) (integer) 500
  2) (integer) 700
  ```

- Redis 事务命令
  - `discard`: 取消事务，放弃执行事务块内的所有命令。
  - `exec`:  执行所有事务块内的命令。
  - `watch key [...]`: 监视一个或多个key集合 ，如果在事务执行之前这个(或这些) key 被其他命令所改动，那么事务将被打断。 
  - `unwatch` : 取消 WATCH 命令对所有 key 的监视。



## 0.7. 数据备份与恢复

