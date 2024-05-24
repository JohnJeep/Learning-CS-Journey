# 慢SQL

慢 SQL 产生的表现：

- 索引缺失
- 索引失效
- 无有效查询条件或条件缺失
- 多层临时表嵌套
- 强制使用性能差的索引

对线上环境产生的影响：出现事故偶发、用户体验差。

对数据库产生的影响：SQL 执行的越慢，消耗 CPU 资源或 IO 资源也会越大，大量的慢 SQL 查询可直接引发业务故障。

## 开启慢SQL查询日志

> MyQL 默认是没有开启慢查询日志的，可以通过命令行或者修改 `my.cnf` 来开启。开启后对性能有一定的影响，生产环境**不建议开启**。
>
> 使用命令行开启，重启服务后慢查询就会**失效**；修改配置文件的方式，会一直**生效**。



```sh
首先是进入到 MySQL 的服务中
查看下关于慢查询的配置 show variables like 'slow_query_log%';

查看下默认的超时时间配置 show variables like 'long_query_time%';
```

命令行中修改

```shell
# 执行步骤
先将超时时间修改为 3 秒 set global long_query_time=3;
然后开启配置 set global slow_query_log=1;
退出 MySQL 并重新进入
测试一下 select sleep(5);
去看下慢查询的日志
```

### 修改MySQL配置的方式

修改 `my.cnf`，在 `[mysqld]` 添加下面配置

```shell
slow_query_log=1
slow_query_log_file=/var/lib/mysql/slow-log.log
long_query_time=3
```

## References

- mysql cpu 占用超过 100%：https://xie.infoq.cn/article/a274e214af9b43f77119b1d90