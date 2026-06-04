<!--
 * @Author: JohnJeep
 * @Date: 2026-03-12 10:52:23
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-04-25 18:29:34
 * @Description: MongoDB Usage
 * Copyright (c) 2026 by John Jeep, All Rights Reserved. 
-->


# 常见系统命令

```bash
# 启动 mongod 进程
sudo systemctl start mongod

# 验证 MongoDB 是否已成功启动
sudo systemctl status mongod

# 设置 mongodb 为开机启动
sudo systemctl enable mongod

# 停止 mongod
sudo systemctl stop mongod

# 重启 mongod 
sudo systemctl restart mongod

# 停止 mongodb
sudo service mongod stop

# 删除之前安装的所有 MongoDB 包
sudo apt-get purge mongodb-org*

# 删除 MongoDB 数据库和日志文件。
sudo rm -r /var/log/mongodb
sudo rm -r /var/lib/mongodb

```

# mongosh

在与 [`mongod`](https://www.mongodb.com/zh-cn/docs/manual/reference/program/mongod/#mongodb-binary-bin.mongod) 相同的主机上启动 [`mongosh`](https://www.mongodb.com/zh-cn/docs/mongodb-shell/#mongodb-binary-bin.mongosh) 会话。您可以在不使用任何命令行选项的情况下运行 [`mongosh`](https://www.mongodb.com/zh-cn/docs/mongodb-shell/#mongodb-binary-bin.mongosh)，从而连接在本地主机上运行且默认端口号为 27017 的 [`mongod`](https://www.mongodb.com/zh-cn/docs/manual/reference/program/mongod/#mongodb-binary-bin.mongod)。

```bash
# 本地连接（默认端口27017）
mongo

# 指定主机和端口
mongo --host 127.0.0.1 --port 27017

# 指定IP和port认证的连接
mongo --host x.x.x.x --port xxxx -u username -p password --authenticationDatabase admin
```



# 数据库查看命令

```bash
# 查看所有数据库
show dbs

# 切换到指定数据库
use database_name

# 查看当前数据库
db

# 查看当前数据库的所有集合
show collections

show tables

# 查看集合中的数据
db.yourCollectionName.find()
db.collection_name.find().pretty()

# 例如
db.shadow.find({sn:"xxxx","productName":"aaa"})

# 查看数据库统计信息
db.stats()
```


# 查看配文件

```bash
# 查看MongoDB配置文件
cat /etc/mongod.conf

# 查看数据目录
cat /etc/mongod.conf | grep dbPath

# 查看日志路径
cat /etc/mongod.conf | grep logPath
```



# 命令行工具

```bash
# 查看实时性能统计
mongostat

# 指定间隔时间（每秒一次）
mongostat 5

# 查看每个集合的读写活动
mongotop

# 指定间隔时间
mongotop 5
```



# References

- mongodb doc: https://www.mongodb.com/zh-cn/docs/
- MongoDB Shell: https://www.mongodb.com/zh-cn/docs/mongodb-shell/
- [Ubuntu 社区版安装](https://www.mongodb.com/zh-cn/docs/manual/administration/install-community/?operating-system=linux&linux-distribution=ubuntu&linux-package=default&macos-installation-method=None&windows-installation-method=None&search-linux=without-search-linux&search-docker=None)