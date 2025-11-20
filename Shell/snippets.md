<!--
 * @Author: JohnJeep
 * @Date: 2024-10-10 15:31:52
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-11-20 12:43:49
 * @Description: 常用代码片段 
 * Copyright (c) 2025 by John Jeep, All Rights Reserved. 
-->
- [1. Influxdb](#1-influxdb)
- [2. find](#2-find)
- [3. mysql](#3-mysql)
- [4. shell](#4-shell)
- [5. docker](#5-docker)


# 1. Influxdb

```sql
// 启动
service influxdb start

// 远程登录
influx -host 172.26.165.132 -port 8086 -username admin -password 123456 -precision rfc3339

// UTC时间戳显示 本地登录
influx -precision rfc3339 -username iottest -password Lanyou@2020

// 查询表
show measurements;

// 删除名为 cpu_usage 的测量表及其所有数据
DROP MEASUREMENT "cpu_usage"

// 查最新多少条；升序：ASC
select * from "RT-D-ZX-214" order by time DESC limit 500;


// 按照时间段查询
select * from "RT-D-ZX-214" where time >= '2024-07-19T15:07:00Z' and time < '2024-07-22T08:00:00Z';

# 导出
influx_inspect export -datadir "/home/iot-services/influxdb/data" -waldir "/home/iot-services/influxdb/wal" -out "influxdb_foundry_out" -database "iotdata" -start "2024-05-03T00:00:00Z"

# 导入
influx -import -path=/root/influxdb_dump_out -precision=ns

其中：
  import: 标识导入
  path: 导入文件
  precision: 导入的数据时间精度
```



# 2. find

```shell
# 删除最近一小时外的日志
find ./*.log -type f -mmin +60 -exec rm -rf {} \;

# 删除超过七天的日志
find ./*.log -type f -mtime +7 -exec rm -rf {} \;

```

# 3. mysql

```shell
// dump database
# 导出整个库，导出时不包含 GTID 信息
mysqldump --set-gtid-purged=OFF -u username -p source_db > source_db.sql

# 查看所有数据库的排序规则 
SELECT 
    SCHEMA_NAME, 
    DEFAULT_CHARACTER_SET_NAME, 
    DEFAULT_COLLATION_NAME
FROM 
    information_schema.SCHEMATA;
    
# 查看当前数据库的排序规则
SELECT 
    SCHEMA_NAME, 
    DEFAULT_CHARACTER_SET_NAME, 
    DEFAULT_COLLATION_NAME
FROM 
    information_schema.SCHEMATA
WHERE 
    SCHEMA_NAME = 'your_database_name';

# 查看某个库中所有表的排序规则
SELECT 
    TABLE_NAME,
    TABLE_COLLATION
FROM 
    information_schema.TABLES
WHERE 
    TABLE_SCHEMA = 'database_name';


# 查看某个表的排序规则
SHOW TABLE STATUS WHERE Name = 'your_table_name';

# 查看某张表中所有字段的排序方式
SHOW FULL COLUMNS FROM table_name;

# 修改某个库排序规则
ALTER DATABASE thingcross CHARACTER SET utf8mb4 COLLATE utf8mb4_0900_ai_ci;

# 修改表中字段排序规则
ALTER TABLE table_name CONVERT TO CHARACTER SET utf8mb4 COLLATE utf8mb4_0900_ai_ci;

```



# 4. shell

```shell
# 取当前系统中状态为“UP”的网络接口的IP地址
ip addr | grep 'state UP' -A2 | tail -n1 | awk '{print $2}' | cut -f1 -d '/'

# 批量杀死进程
ps aux | grep xxx | grep -v grep | awk '{print "kill -9 " $2}' | sh
```

# 5. docker

```shell
# 镜像导出
docker save -o images.tar images:tag

# 镜像导入
docker load images.tar
```

