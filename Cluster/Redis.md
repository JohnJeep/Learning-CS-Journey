<!--
 * @Author: JohnJeep
 * @Date: 2020-09-05 23:51:27
 * @LastEditTime: 2020-09-21 11:12:39
 * @LastEditors: Please set LastEditors
 * @Description: redis学习
-->
## 参考
- [redis.io](https://redis.io/) : Redis官方网站
- [redis.cn](http://redis.cn/): Redis中文网站
- [redis.cn-commands](http://redis.cn/commands.html) redis中文版相关命令用法
- [Redis学习教程](https://piaosanlang.gitbooks.io/redis/content/index.html) 比较全面的介绍了有关redis的使用。



## 概念
- 什么是redis？
  > Redis是一个开源的使用ANSI C语言编写、遵守BSD协议、支持网络、可基于内存亦可持久化的日志型、key-Value 的数据库、并提供多种语言的API。通常，Redis 将数据存储于内存中，或被配置为使用虚拟内存。

- Redis的主要优点
  - Redis支持数据的持久化，可以将内存中的数据保持在磁盘中，重启的时候可以再次加载进行使用
  - Redis不仅仅支持简单的 `key-value` 类型的数据，同时还提供 `list`，`set`，`zset`，`hash`等数据结构的存储
  - Redis支持数据的备份，即master-slave模式的数据备份。

## 数据对象
### string(字符串)
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

### hash(哈希)
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


### list(列表)
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


### set(集合)
- Redis Set是string类型的无序集合。集合是通过哈希表实现的，所以添加，删除，查找的复杂度都是O(1)。 在 Redis 可以添加，删除和测试成员存在的时间复杂度为 O（1）。
- 集合中最大的成员数为 $2^{32} - 1$ (4294967295, 每个集合可存储40多亿个成员)。
-  Redis 非常人性化的为集合提供了 求交集、并集、差集等操作, 那么就可以非常方便的实现如共同关注、共同喜好、二度好友等功能, 对上面的所有集合操作,你还可以使用不同的命令选择将结果返回给客户端还是存集到一个新的集合中。

    ```
    // sadd: 向集合中添加元素，scard: 返回集合中元素的size 
    // sdiff key1 key2: 返回ke1集合中除去与key2集合中共有的元素
    // sinter key1 key2: 返回key1集合与key2集合的交集的元素

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

- 应用场景
  - 1.共同好友、二度好友
  - 2.利用唯一性,可以统计访问网站的所有独立 IP
  - 3.好友推荐的时候,根据 tag 求交集,大于某个 临界值 就可以推荐

### zset(sorted set 有序集合)
- Redis 有序集合和集合一样也是string类型元素的集合,且不允许重复的成员。



