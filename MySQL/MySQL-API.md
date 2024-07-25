# c++ connect API

```
mysql_affected_rows() 返回被最新的UPDATE, DELETE或INSERT查询影响的行数。
mysql_close() 关闭一个服务器连接。
mysql_connect() 连接一个MySQL服务器。该函数不推荐；使用mysql_real_connect()代替。
mysql_change_user() 改变在一个打开的连接上的用户和数据库。
mysql_create_db() 创建一个数据库。该函数不推荐；而使用SQL命令CREATE DATABASE。
mysql_data_seek() 在一个查询结果集合中搜寻一任意行。
mysql_debug() 用给定字符串做一个DBUG_PUSH。
mysql_drop_db() 抛弃一个数据库。该函数不推荐；而使用SQL命令DROP DATABASE。
mysql_dump_debug_info() 让服务器将调试信息写入日志文件。
mysql_eof() 确定是否已经读到一个结果集合的最后一行。这功能被反对; mysql_errno()或mysql_error()可以相反被使用。
mysql_errno() 返回最近被调用的MySQL函数的出错编号。
mysql_error() 返回最近被调用的MySQL函数的出错消息。
mysql_escape_string() 用在SQL语句中的字符串的转义特殊字符。
mysql_fetch_field() 返回下一个表字段的类型。
mysql_fetch_field_direct () 返回一个表字段的类型，给出一个字段编号。
mysql_fetch_fields() 返回一个所有字段结构的数组。
mysql_fetch_lengths() 返回当前行中所有列的长度。
mysql_fetch_row() 从结果集合中取得下一行。
mysql_field_seek() 把列光标放在一个指定的列上。
mysql_field_count() 返回最近查询的结果列的数量。
mysql_field_tell() 返回用于最后一个mysql_fetch_field()的字段光标的位置。
mysql_free_result() 释放一个结果集合使用的内存。
mysql_get_client_info() 返回客户版本信息。
mysql_get_host_info() 返回一个描述连接的字符串。
mysql_get_proto_info() 返回连接使用的协议版本。
mysql_get_server_info() 返回服务器版本号。
mysql_info() 返回关于最近执行得查询的信息。
mysql_init() 获得或初始化一个MYSQL结构。
mysql_insert_id() 返回有前一个查询为一个AUTO_INCREMENT列生成的ID。
mysql_kill() 杀死一个给定的线程。
mysql_list_dbs() 返回匹配一个简单的正则表达式的数据库名。
mysql_list_fields() 返回匹配一个简单的正则表达式的列名。
mysql_list_processes() 返回当前服务器线程的一张表。
mysql_list_tables() 返回匹配一个简单的正则表达式的表名。
mysql_num_fields() 返回一个结果集合重的列的数量。
mysql_num_rows() 返回一个结果集合中的行的数量。
mysql_options() 设置对mysql_connect()的连接选项。
mysql_ping() 检查对服务器的连接是否正在工作，必要时重新连接。
mysql_query() 执行指定为一个空结尾的字符串的SQL查询。
mysql_real_connect() 连接一个MySQL服务器。
mysql_real_query() 执行指定为带计数的字符串的SQL查询。
mysql_reload() 告诉服务器重装授权表。
mysql_row_seek() 搜索在结果集合中的行，使用从mysql_row_tell()返回的值。
mysql_row_tell() 返回行光标位置。
mysql_select_db() 连接一个数据库。
mysql_shutdown() 关掉数据库服务器。
mysql_stat() 返回作为字符串的服务器状态。
mysql_store_result() 检索一个完整的结果集合给客户。
mysql_thread_id() 返回当前线程的ID。
mysql_use_result() 初始化一个一行一行地结果集合的检索。
```

# 同步

同步连接池维护的是短链接，

- connect_times 连接次数
- read_timeout
- write_timeout

需要考虑连接池的伸缩性问题

- 最小连接数：限制池的大小。
- 最大连接数：限制打开的连接数量。



同步连接池数量的初始大小如何设置？

​	若 CPU 的核心数为 8 核，则数量为：从 `2*n + 2` 开始测试，直到找到最优的数。

# 异步

回调函数接收 DB的返回值



**异步连接池维护的是长连接。**

怎样实现异步的连接

同步连接池需要创建多个线程。

异步连接不需要创建额外的线程，复用主线程、复用网络模块。

1. 创建一个非阻塞的连接
2. 接入数据库的协议。若数据库提供异步的接口，则只需要适配事件的框架就可以了。



# References

1. [**max_connections**](https://releem.com/docs/mysql-performance-tuning/max_connections)
2. [MySQL服务器最大连接数怎么设置才合理](https://www.cnblogs.com/helloyb/p/5025782.html)