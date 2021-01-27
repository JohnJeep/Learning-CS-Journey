<!--
 * @Author: JohnJeep
 * @Date: 2020-09-05 23:51:27
 * @LastEditTime: 2021-01-27 21:43:35
 * @LastEditors: Please set LastEditors
 * @Description: redis学习
-->

<!-- TOC -->

- [0.1. 参考](#01-参考)
- [0.2. 概念](#02-概念)
- [0.3. 启动客户端与服务器](#03-启动客户端与服务器)
- [0.4. KEYS命令](#04-keys命令)
  - [0.4.1. KEYS pattern](#041-keys-pattern)
  - [0.4.2. EXISTS命令](#042-exists命令)
  - [0.4.3. DEL命令](#043-del命令)
  - [0.4.4. DUMP命令](#044-dump命令)
  - [0.4.5. MOVE命令](#045-move命令)
  - [0.4.6. TYPE命令](#046-type命令)
  - [0.4.7. RENAME命令](#047-rename命令)
  - [0.4.8. 生存时间](#048-生存时间)
- [0.5. 基本数据类型](#05-基本数据类型)
  - [0.5.1. string(字符串)](#051-string字符串)
  - [0.5.2. hash(哈希)](#052-hash哈希)
  - [0.5.3. list(列表)](#053-list列表)
  - [0.5.4. set(集合)](#054-set集合)
  - [0.5.5. zset(sorted set 有序集合)](#055-zsetsorted-set-有序集合)
- [0.6. pub/sub(订阅与发布模式)](#06-pubsub订阅与发布模式)
- [0.7. 事务](#07-事务)
- [0.8. redis连接](#08-redis连接)
- [0.9. redis服务器](#09-redis服务器)
- [0.10. HyperLogLog](#010-hyperloglog)

<!-- /TOC -->


## 0.1. 参考
- [redis.io](https://redis.io/) : Redis官方网站
- [redis.cn-commands](http://redis.cn/commands.html)：redis中文版相关命令用法
- [Redis学习教程](https://piaosanlang.gitbooks.io/redis/content/index.html)：比较全面的介绍了有关redis的使用。
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



## 0.3. 启动客户端与服务器
- `redis-cli`：启动Redis 客户端
- `redis-cli -h host -p port -a password`：远程机器上启动Redis 客户端
- `redis-server`：启动Redis 服务器
```
$redis-cli -h 127.0.0.1 -p 6379 -a "mypass"
redis 127.0.0.1:6379>
redis 127.0.0.1:6379> PING   // 检测 redis 服务是否启动

PONG
```


## 0.4. KEYS命令
> Redis 键命(key)令用于管理 redis 的键。

### 0.4.1. KEYS pattern
- `key pattern`: 查找所有匹配给定模式的键。
- 示例：`keys *` : 查询数据库中所有的键。
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


### 0.4.3. DEL命令
- `del key1, key2 ...`: 删除一个或多个按键，返回值是删除的键的个数。
  ```
  127.0.0.1:6379> del match_rank s_set
  (integer) 2
  ```


### 0.4.4. DUMP命令
- `dump key`：序列化给定的key ，如果 key 不存在，那么返回 nil； 否则，返回序列化之后的值。
- 序列化生成的值有以下几个特点：
  - 它带有 64 位的校验和，用于检测错误，RESTORE 在进行反序列化之前会先检查校验和。
  - 值的编码格式和 RDB 文件保持一致。
  - RDB 版本会被编码在序列化值当中，如果因为 Redis 的版本不同造成 RDB 格式不兼容，那么 Redis 会拒绝对这个值进行反序列化操作。
  - 序列化的值不包括任何生存时间信息。

  ```
  127.0.0.1:6379> get name
  "jackpeter"
  127.0.0.1:6379> dump name
  "\x00\tjackpeter\x06\x00\x94\\\xe0\xa8>\x0ef\xf8"
  127.0.0.1:6379> dump na
  (nil)
  ```


### 0.4.5. MOVE命令
- `move key db`：将当前数据库的键值 key 移动到给定的数据库 db 当中，移动成功返回 1，失败则返回 0。
- 如果当前数据库(源数据库)和给定数据库(目标数据库)有相同名字的给定 key ，或者 key 不存在于当前数据库，那么 MOVE 没有任何效果。


### 0.4.6. TYPE命令
- `type key1, key2, ...`: 获取按键的数据类型
  - 数据类型可以是string、hash、list、set、zset。

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

### 0.4.7. RENAME命令
- `rename key key1`：将键值key修改为key1
- `renamenx key key1`：仅当键值key1不存在当前数据库中时，才可以将key修改为key1
- `randomkey`：从当前数据库中随机返回一个 key 。


```
127.0.0.1:6379> type name
string
127.0.0.1:6379> rename name my_name
OK
127.0.0.1:6379> renamenx my_name age   // 数据库中已存在age键
(integer) 0
127.0.0.1:6379> get my_name
"jackpeter"
127.0.0.1:6379> renamenx my_name tt
(integer) 1
127.0.0.1:6379> randomkey
"key2"
127.0.0.1:6379> randomkey
"age"
```


### 0.4.8. 生存时间
- redis在实际使用过程中一般用作缓存，然而缓存的数据一般都需要设置生存时间，也就是到期后销毁数据。
- `expire key seconds`：键值对的生存时间为 seconds，时间到后，销毁键值对中的数据
- `TTL key`：以秒为单位，返回给定 key 的剩余生存时间(TTL, time to live)。
  - TTL返回值：
    - 大于0的数字：剩余生存时间，单位为秒
    - -1 ： 没有生存时间，永久存储
    - -2 ： 数据已经被删除
- `pttl key`：以毫秒为单位返回 key 的剩余的过期时间。
- `persist key` 清除键值对的生存时间，key将永久保持。注意：重新设置值也会清除生存时间。


```
127.0.0.1:6379> set my_live time
OK
127.0.0.1:6379> get my_live
"time"
127.0.0.1:6379> ttl my_live
(integer) -1
127.0.0.1:6379> expire my_live 20
(integer) 1
127.0.0.1:6379> ttl my_live
(integer) 18
127.0.0.1:6379> ttl my_live
(integer) 14
127.0.0.1:6379> ttl my_live
(integer) 1
127.0.0.1:6379> ttl my_live
(integer) -2
127.0.0.1:6379> get my_live
(nil)

127.0.0.1:6379> set my_live time
OK
127.0.0.1:6379> ttl my_live
(integer) -1
127.0.0.1:6379> expire my_live 2000
(integer) 1
127.0.0.1:6379> ttl my_live
(integer) 1996
127.0.0.1:6379> ttl my_live
(integer) 1993
127.0.0.1:6379> get my_live
"time"
127.0.0.1:6379> persist my_live
(integer) 1
127.0.0.1:6379> ttl my_live
(integer) -1
127.0.0.1:6379> get my_live
"time"
```



## 0.5. 基本数据类型
### 0.5.1. string(字符串)
- string类型是Redis最基本的数据类型，一个 `key`对应一个 `value` ，一个键最大能存储512MB。

- 基础命令
  - `set key value`：设置key对应的value 
  - `mset key value [key value]`：设置多个key对应的value 
  - `get key`：获取key对应的value
  - `mget key`：获取多个key对应的value
  - `incr key`：当value值为整数时，将当前key对应的键值每次加一，并返回递增后的值。
  - `incrby key num`：当value值为整数时，将当前key对应的键值每次加 num，并返回递增后的值。
  - `incrbyfloat key num`：当value值为浮点数时，将当前key对应的键值每次加 num，并返回递增后的值。
  - `decr key`：将当前key对应的键值每次减一，并返回递减后的值。
  - `decrby key num`：将当前key对应的键值每次减 num，并返回递减后的值。
  - `append key value`：向原来key对应的value 末尾添加value，返回追加后字符串的总长度。
  - `strlen key`：获取key对应的value的长度。 若键值不存在则返回0
  - `getrange key start end`：获取key对应的value中的子字符串，start和end为value的索引下标，从0开始。
  - `getbit key offset`：返回key对应的value在字符串中offset偏移位置处的bit值（存在为1，不存在为0）。


  ```
  redis 127.0.0.1:6379> SET name "Jack"
  OK
  redis 127.0.0.1:6379> GET name
  "Jack"
  127.0.0.1:6379> type name
  string
  127.0.0.1:6379> mset age 20 sex man  score 100
  OK
  127.0.0.1:6379> mget age sex score
  1) "20"
  2) "man"
  3) "100"

  127.0.0.1:6379> get wang
  "700"
  127.0.0.1:6379> incr wang
  (integer) 701
  127.0.0.1:6379> get wang
  "701"

  127.0.0.1:6379> incrby wang 10
  (integer) 713
  127.0.0.1:6379> incrby wang 12
  (integer) 725
  127.0.0.1:6379> decr wang
  (integer) 724
  127.0.0.1:6379> decr wang
  (integer) 723
  127.0.0.1:6379> decrby wang 5
  (integer) 718
  127.0.0.1:6379> decrby wang 3
  (integer) 715

  127.0.0.1:6379> append score 10
  (integer) 5
  127.0.0.1:6379> get score
  "10010"
  127.0.0.1:6379> strlen score
  (integer) 5

  127.0.0.1:6379> get name
  "jack"
  127.0.0.1:6379> append name peter
  (integer) 9
  127.0.0.1:6379> get name
  "jackpeter"
  127.0.0.1:6379> getrange name 0 3
  "jack"

  ```


### 0.5.2. hash(哈希)
- Redis hash是一个string类型的 `field` 和 `value` 的映射表，是一个键值对集合，hash特别适合用于存储对象。
<img src="./figures/redis-hash结构.png">


- 基础命令
  - `hset key field value`：向键值为key的hash中添加字段为field，值为value的数据。
  - `hsetnx key field value`：向键值为key的hash中添加字段为field，值为value的数据，只有当这个字段不存在时才有效。
    - 返回1：如果字段是个新的字段，并成功赋值
    - 返回0：如果哈希集中已存在该字段，没有操作被执行  
  - `hget key field`：从hash表中返回键值为key，字段为field的value值。
  - `hmset key field value [field value]`  向键值为key的hash中添加多个字段为field，值为value的数据。
  - `hmget key field [field]`：从hash表中返回键值为key，多个字段为field的value值。
  - `hgetall key`：得到hash集合中所有的键值key和值value。
  - `hkeys key`：获取hash中所有的field值。
  - `hvals key`：获取hash中所有的value值。
  - `hlen key`：获取hash中所有field的数量。
  - `hexists key field`： 判断键值为key的hash表中是否存在字段field。
  > 注意：hset命令不区分插入和更新操作，当执行插入操作时，hset返回结果为1，当执行更新操作时，返回结果为0

  - `hincrby key field num`：每次向键值为key，字段为field的hash中添加 num
  - `hincrbyfloat key field num`：每次向键值为key，字段为field的hash中添加浮点数 num
  - `hdel key field [field ...]`：删除hash表中键值为key中的一个或多个字段field，返回的结果为被删除字段field的个数。
  - `hscan key cursor  [MATCH pattern] [COUNT count]`：迭代哈希表中的键值对。



  ```
  // 往hash表中插入数据，user是一个键值key
  127.0.0.1:6379> hset user username zhangsan
  (integer) 1
  127.0.0.1:6379> hget user username
  "zhangsan"
  127.0.0.1:6379> hmset user password 123
  OK
  127.0.0.1:6379> hmset user name ZS age 23
  OK
  127.0.0.1:6379> hgetall user
  1) "username"
  2) "zhangsan"
  3) "password"
  4) "123"
  5) "name"
  6) "ZS"
  7) "age"
  8) "23"
  127.0.0.1:6379> hmget user age name
  1) "23"
  2) "ZS"
  127.0.0.1:6379> hkeys user
  1) "username"
  2) "password"
  3) "name"
  4) "age"

  127.0.0.1:6379> hexists user age
  (integer) 1
  127.0.0.1:6379> hexists user sex
  (integer) 0
  127.0.0.1:6379>

  127.0.0.1:6379> hlen user
  (integer) 4
  127.0.0.1:6379> hvals user
  1) "zhangsan"
  2) "123"
  3) "ZS"
  4) "23"

  127.0.0.1:6379> hsetnx user name jock
  (integer) 0
  127.0.0.1:6379> hsetnx user sex woman
  (integer) 1

  127.0.0.1:6379> hscan user 2
  1) "0"
  2)  1) "username"
      2) "zhangsan"
      3) "password"
      4) "123"
      5) "name"
      6) "ZS"
      7) "age"
      8) "23"
      9) "sex"
    10) "woman"
  ```


### 0.5.3. list(列表)
- Redis 列表是简单的字符串列表，按照插入顺序排序。你可以添加一个元素到列表的头部（左边）或者尾部（右边）。
- 列表最多可存储 $2^{32} - 1$ 元素 (4294967295, 每个列表可存储40多亿)。

- 基础命令
  - `lpush key value1 [value2, value3 ...]`： 往list列表头部中添加一个或多个数据。
  - `rpush key value1 []value2, ...`：往list列表尾部中添加一个或多个数据。
  - `linsert key before | after pivot value`：把value值插入在key列表的基准值 pivot 的前面或后面。
  
  
  - `lrange key start stop`：获取列表指定范围内的元素
  - `lindex key index`：通过索引获取列表中index处的元素
  - `llen key`：获取列表长度
  - `lrem key count value`：移除列表中出现count次，值为value的元素
    - count > 0: 从头往尾移除值为 value 的元素。
    - count < 0: 从尾往头移除值为 value 的元素。
    - count = 0: 移除所有值为 value 的元素。


  - `lpop key`：移出并获取列表的第一个元素
  - `rpop key`：移出并获取列表的最后一个元素
  - `blpop key1 [key2] timeout`：移出并获取列表的第一个元素， 如果列表没有元素，会阻塞列表直到等待超时或发现可弹出元素为止。
  - `brpop key1 [key2] timeout`：移除列表中的最后一个元素，返回一个双元素的多批量值，其中第一个元素是弹出元素的 key，第二个元素是 value。如果列表没有元素，会阻塞列表直到等待超时或发现可弹出元素为止。
  - `brpoplpush source destination timeout`：从source列表中弹出最右边的值，将弹出的元素从最左边插入到另外一个destination列表中并返回它；如果列表没有元素，会阻塞列表直到等待超时或发现可弹出元素为止。
  - `rpoplpush source destination`：从source列表中弹出最右边的值，将弹出的元素从最左边插入到另外一个destination列表中并返回它；

  - `lset key index value`：通过index去改变list中的值
  - `ltrim key start stop`：让列表只保留指定区间内的元素，不在指定区间之内的元素都将被删除。  
  


  ```
  127.0.0.1:6379> type my_list
  list
  127.0.0.1:6379> llen my_list
  (integer) 4
  127.0.0.1:6379> lrange my_list 0 3
  1) "peach"
  2) "pear"
  3) "banana"
  4) "apple"
  127.0.0.1:6379> lindex my_list 2
  "banana"

  127.0.0.1:6379> lpush my_list  aa aa bb aa bb
  (integer) 9
  127.0.0.1:6379> lrange my_list 0 8
  1) "bb"
  2) "aa"
  3) "bb"
  4) "aa"
  5) "aa"
  6) "peach"
  7) "pear"
  8) "banana"
  9) "apple"
  127.0.0.1:6379> lrem my_list 2 aa
  (integer) 2
  127.0.0.1:6379> lrange my_list 0 6
  1) "bb"
  2) "bb"
  3) "aa"
  4) "peach"
  5) "pear"
  6) "banana"
  127.0.0.1:6379> llen my_list
  (integer) 7

  127.0.0.1:6379> blpop my_list 3
  1) "my_list"
  2) "bb"
  127.0.0.1:6379> lrange my_list 0 5
  1) "bb"
  2) "aa"
  3) "peach"
  4) "pear"
  5) "banana"
  6) "apple"
  127.0.0.1:6379> brpop my_list 3
  1) "my_list"
  2) "apple"

  127.0.0.1:6379> brpoplpush my_list temp_list 5
  "banana"
  127.0.0.1:6379> lrange my_list 0 5
  1) "bb"
  2) "aa"
  3) "peach"
  4) "pear"
  127.0.0.1:6379> lrange temp_list 0 5
  1) "banana"

  127.0.0.1:6379> lset my_list 1 hello
  OK
  127.0.0.1:6379> lrange my_list 0 5
  1) "bb"
  2) "hello"
  3) "bb"
  4) "aa"
  5) "peach"
  6) "pear"

  127.0.0.1:6379> rpush my_list gg
  (integer) 5
  127.0.0.1:6379> lrange my_list 0 5
  1) "bb"
  2) "hello"
  3) "bb"
  4) "aa"
  5) "gg"

  127.0.0.1:6379> lrange my_list 0 5
  1) "bb"
  2) "hello"
  3) "bb"
  4) "aa"
  5) "peach"
  6) "pear"
  127.0.0.1:6379> rpop my_list
  "pear"
  127.0.0.1:6379> rpoplpush my_list temp_list
  "peach"
  ```


### 0.5.4. set(集合)
- Redis Set是string类型的无序集合。集合是通过哈希表实现的，所以添加，删除，查找的复杂度都是O(1)。 在 Redis 可以添加，删除和测试成员存在的时间复杂度为 O（1）。
- 集合中最大的成员数为 $2^{32} - 1$ (4294967295, 每个集合可存储40多亿个成员)。
-  Redis 非常人性化的为集合提供了 求交集、并集、差集等操作, 那么就可以非常方便的实现如共同关注、共同喜好、二度好友等功能, 对上面的所有集合操作,你还可以使用不同的命令选择将结果返回给客户端还是存集到一个新的集合中。

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

    - `srem my_set member [...]` 从my_set集合中删除指定的一个或多个元素。
    - `sunion my_set1 my_set2` 返回my_set1集合与my_set2集合中所有元素的并集。
    - `sunionstore set_key set_key1 set_key2` 返回set_key1集合与set_key2集合中所有元素的并集，并将结果存储在新的集合set_key中。
    - `sscan my_set ` 迭代当前数据库中my_key集合的元素

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

- 应用场景
  - 1.共同好友、二度好友
  - 2.利用唯一性,可以统计访问网站的所有独立 IP
  - 3.好友推荐的时候,根据 tag 求交集,大于某个 临界值 就可以推荐


### 0.5.5. zset(sorted set 有序集合)
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


## 0.6. pub/sub(订阅与发布模式)
> Redis 发布订阅(pub/sub)是一种消息通信模式：发送者(publish)发送消息，订阅者(subscribe)接收消息。 Redis 客户端可以订阅任意数量的频道。

- Redis 发布订阅命令
  - `psubscribe pattern [pattern ...]`: 订阅一个或多个符合给定模式的频道，返回接收到的信息。
  - `pubsub subcommand [argument [argument ...]]`: 查看订阅与发布系统状态。
  - `subscribe channel [channel ...]`: 订阅给定的一个或多个频道的信息。
  - `unsubscribe [channel [channel] ...]`: 退订指定的频道。
  - `punsubsrcibe [pattern [pattern] ...]`: 停止发布到指定模式的频道。
  - `publish channel message`: 给频道channel发布一条消息，返回收到消息的客户端数量。 




## 0.7. 事务
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
  - `discard`: 取消事务，丢弃所有multi之后发的命令，所有返回都是 OK。
  - `exec`：执行所有multi之后发的命令。
  - `multi`：标记一个事务块的开始，标记一个事务块的开始。
  - `unwatch` : 取消 WATCH 命令对所有键值key 的监视。 
  - `watch key [...]`: 锁定键值key，直到执行列multi、exec命令。如果在事务执行之前一个或多个 key 被其他命令所改动，那么事务将被打断。



## 0.8. redis连接
>  Redis中，一共有16个数据库，分别是0~15，一般情况下，进入数据库默认编号是0

- `select index`：切换到指定的index数据库
- `quit`：关闭当前连接
- `ping`：查看服务是否运行
- `echo msg`：打印字符串
- `auth psd`：验证密码是否正确
- `swapdb index1 index2`：交换同一Redis服务器上两个数据库的数据，执行成功返回OK 。


## 0.9. redis服务器
- `dbsize`：返回当前数据库中所有key的数目
- `flushdb`：删除当前数据库中的所有key
- `flushall`：清空所有数据库中的所有key
- `bgsave`：在后台异步保存当前数据库的数据到磁盘，会立即返回 OK 状态码。edis forks, 父进程继续提供服务以供客户端调用，子进程将DB数据保存到磁盘然后退出。如果操作成功，可以通过客户端命令 `lastsave` 来检查操作结果。
- `client list`：列出所有已连接客户端信息和统计数据。
  - `id`: 唯一的64位的客户端ID(Redis 2.8.12加入)。
  - `addr`: 客户端的地址和端口
  - `fd`: 套接字所使用的文件描述符
    - r: 客户端套接字（在事件 loop 中）是可读的（readable）
    - w: 客户端套接字（在事件 loop 中）是可写的（writeable） 
  - `age`: 以秒计算的已连接时长
  - `idle`: 以秒计算的空闲时长
  - `flags`: 客户端 flag。
  - `db`: 该客户端正在使用的数据库 ID
  - `sub`: 已订阅频道的数量
  - `psub`: 已订阅模式的数量
  - `multi`: 在事务中被执行的命令数量
  - `qbuf`: 查询缓冲区的长度（字节为单位， 0 表示没有分配查询缓冲区）
  - `qbuf-free`: 查询缓冲区剩余空间的长度（字节为单位， 0 表示没有剩余空间）
  - `obl`: 输出缓冲区的长度（字节为单位， 0 表示没有分配输出缓冲区）
  - `oll`: 输出列表包含的对象数量（当输出缓冲区没有剩余空间时，命令回复会以字符串对象的形式被入队到这个队列里）
  - `omem`: 输出缓冲区和输出列表占用的内存总量
  - `events`: 文件描述符事件
  - `cmd`: 最近一次执行的命令

  > 客户端 flag 可以由以下部分组成：
  ```
  O: 客户端是 MONITOR 模式下的附属节点（slave）
  S: 客户端是一般模式下（normal）的附属节点
  M: 客户端是主节点（master）
  x: 客户端正在执行事务
  b: 客户端正在等待阻塞事件
  i: 客户端正在等待 VM I/O 操作（已废弃）
  d: 一个受监视（watched）的键已被修改， EXEC 命令将失败
  c: 在将回复完整地写出之后，关闭链接
  u: 客户端未被阻塞（unblocked）
  U: 通过Unix套接字连接的客户端
  r: 客户端是只读模式的集群节点
  A: 尽可能快地关闭连接
  N: 未设置任何 flag
  ```

- `client id`：返回当前连接的ID；Redis 5 新增的命令。
- `client setname connection-name`：为当前连接分配一个名字connection-name，这个名字会显示在CLIENT LIST命令的结果中，用于识别当前正在与服务器进行连接的客户端。
- `client getname`：返回当前连接由 CLIENT SETNAME设置的名字。如果没有用CLIENT SETNAME设置名字，将返回一个空的值。
- `client kill addr:port`：关闭指定地址和端口号的客户端
- `client pause timeout`：将所有客户端的访问暂停给定的毫秒数
  > 可以在MULTI/EXEC中一起使用CLIENT PAUSE 和INFO replication以在阻塞的同时获取当前master的偏移量。用这种方法，可以让slaves处理至给定的复制偏移节点。
- `info [section]`：返回关于Redis服务器的各种信息和统计数值。通过给定可选的参数 section ，可以让命令只返回某一部分的信息:
  - server: Redis服务器的一般信息
  - clients: 客户端的连接部分
  - memory: 内存消耗相关信息
  - persistence: RDB和AOF相关信息
  - stats: 一般统计
  - replication: 主/从复制信息
  - cpu: 统计CPU的消耗
  - commandstats: Redis命令统计
  - cluster: Redis集群信息
  - keyspace: 数据库的相关统计

- `save`：执行一个同步操作，以RDB文件的方式保存所有数据的快照 很少在生产环境直接使用SAVE 命令，因为它会阻塞所有的客户端的请求，可以使用BGSAVE 命令代替。
- `lastsave`：获得租后一次磁盘同步的时间。执行成功时返回UNIX时间戳。客户端执行 BGSAVE 命令时，可以通过每N秒发送一个 LASTSAVE 命令来查看BGSAVE 命令执行的结果，由 LASTSAVE 返回结果的变化可以判断执行结果。
- `shutsown [nosave] [save]`：关闭服务器
- 恢复数据：只需将备份文件 (dump.rdb) 移动到 redis 安装目录并启动服务即可。获取 redis 目录可以使用 `CONFIG` 命令。
  ```
  127.0.0.1:6379> save
  OK
  127.0.0.1:6379> config get dir
  1) "dir"
  2) "D:\\Redis-x64-3.0.504"
  127.0.0.1:6379> bgsave
  Background saving started
  ```

- `command`：以数组的形式返回有关所有Redis命令的详细信息。
- `command count`：返回Redis服务器命令的总数。



## 0.10. HyperLogLog
- Redis HyperLogLog 是用来做基数统计的算法，HyperLogLog 的优点是，在输入元素的数量或者体积非常非常大时，计算基数所需的空间总是固定 的、并且是很小的。

- 在 Redis 里面，每个 HyperLogLog 键只需要花费 12 KB 内存，就可以计算接近 $2^{64}$ 个不同元素的基 数。这和计算基数时，元素越多耗费内存就越多的集合形成鲜明对比。但是，因为 HyperLogLog 只会根据输入元素来计算基数，而不会储存输入元素本身，所以 HyperLogLog 不能像集合那样，返回输入的各个元素。

- 什么是基数?
  > 比如数据集 {1, 3, 5, 7, 5, 7, 8}， 那么这个数据集的基数集为 {1, 3, 5 ,7, 8}, 基数(不重复元素)为5。 基数估计就是在误差可接受的范围内，快速计算基数。 

- Redis HyperLogLog 命令
  - `pfadd key element [elelment ..]`： 添加指定元素到 HyperLogLog 中。
  - `pfcount key [key ..]`：返回给定 HyperLogLog 的基数估算值。
  - `pfmerge destkey sourcekey [sourcekey]`： 将多个 HyperLogLog 合并（merge）为一个 HyperLogLog ， 合并后的 HyperLogLog 的基数接近于所有输入 HyperLogLog 的可见集合（observed set）的并集.

  ```
  127.0.0.1:6379> pfadd my_log 11 22 33 44
  (integer) 1
  127.0.0.1:6379> pfadd you_log 99 88 77 66
  (integer) 1
  127.0.0.1:6379> pfcount my_log you_log
  (integer) 8
  127.0.0.1:6379> pfcount my_log
  (integer) 4
  127.0.0.1:6379> pfmerge all_log you_log my_log
  OK
  127.0.0.1:6379> pfcount all_log
  (integer) 8
  ```
