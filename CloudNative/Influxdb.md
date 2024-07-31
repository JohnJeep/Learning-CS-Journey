# 1. InfluxDB 

InfluxDB 是一个高性能的时序数据库，专门用于处理高写入负载的数据存储，尤其是时间序列数据，比如物联网传感器数据、性能监控数据和实时分析数据等。以下是InfluxDB的一些基本概念和操作介绍：

### 什么是InfluxDB？

InfluxDB 是由InfluxData公司开发的一款开源时序数据库（Time Series Database，TSDB），设计用于高性能的写入和查询操作，特别是针对时间序列数据。它的主要特点包括：

- **高性能写入和查询**：适用于高吞吐量的数据写入和快速的查询操作。
- **Schemaless（无模式）**：不需要预定义数据模式，数据可以灵活地添加。
- **内置的数据压缩和持久化**：有效地存储和管理大量数据。
- **丰富的查询语言（InfluxQL）**：提供类SQL的查询语言，便于数据操作和分析。

### 基本概念

1. **Database（数据库）**：InfluxDB中数据的集合，每个数据库包含多个测量、序列和数据点。
2. **Measurement（测量）**：相当于传统数据库中的表，用来存储一类数据。
3. **Tag（标签）**：用于数据的元数据，类似于数据库中的索引，帮助快速查询和过滤数据。标签是字符串类型。
4. **Field（字段）**：存储实际数据，字段可以是多种类型（整数、浮点数、字符串等）。
5. **Timestamp（时间戳）**：每个数据点都有一个时间戳，表示数据记录的时间。

# 2. 连接InfluxDB

```sql
influx --username 'admin' --password 'password'
```

## 2.1. 用户权限管理

1. 创建管理员用户

首先，你需要创建一个具有管理员权限的用户。这通常是在首次设置InfluxDB时完成的。如果你还没有创建管理员用户，可以使用以下命令：

```sql
CREATE USER admin WITH PASSWORD 'adminpassword' WITH ALL PRIVILEGES
```

2. 启用认证

确保在InfluxDB配置文件中启用了认证。配置文件通常位于 `/etc/influxdb/influxdb.conf`（Linux）或 `/usr/local/etc/influxdb.conf`（MacOS）。

找到 `[http]` 部分，并确保以下行被设置：

```conf
[http]
  auth-enabled = true
```

重启InfluxDB服务：

```bash
service influxdb restart
```

3. 创建普通用户并授予权限

如果你已经有一个普通用户，例如 `iottest`，你需要为该用户授予特定的权限。例如，授予 `iotdata` 数据库的读写权限：

```sql
CREATE USER iottest WITH PASSWORD 'password'
GRANT ALL ON iotdata TO iottest
```

如果用户已经存在，只需要授予权限：

```sql
GRANT ALL ON iotdata TO iottest
```

4. 连接并使用认证

确保你在连接InfluxDB时使用了正确的用户名和密码。如果你使用的是CLI，可以在连接时指定用户：

```bash
influx -username 'admin' -password 'adminpassword'
```

然后选择数据库并执行查询：

```sql
USE iotdata
SHOW GRANTS FOR "iottest"
```

5. 验证权限

你可以检查用户的权限设置是否正确：

```sql
SHOW USERS
```

这会显示所有用户及其管理员状态。你还可以使用以下命令来查看特定用户的权限：

```sql
SHOW GRANTS FOR "iottest"
```

## 2.2. influxdb 账户密码忘记

如果你忘记了InfluxDB的账户密码，可以通过以下几种方式解决问题，具体取决于你是否能够直接访问InfluxDB的配置文件和数据文件。

**停用认证临时修改密码**

这是最常用的解决方案，适用于你能够访问InfluxDB的配置文件并且可以重启InfluxDB服务。

步骤：

1. **编辑配置文件**： 打开InfluxDB的配置文件，通常位于 `/etc/influxdb/influxdb.conf`（Linux）或 `/usr/local/etc/influxdb.conf`（MacOS）。

   找到 `[http]` 部分，并将 `auth-enabled` 设置为 `false`：

   ```conf
   [http]
     auth-enabled = false
   ```

2. **重启InfluxDB服务**：

   ```bash
   service influxdb restart
   ```

3. **修改用户密码**： 现在你可以不使用认证直接连接到InfluxDB：

   ```bash
   influx
   ```

   然后修改用户密码：

   ```sql
   USE _internal
   SET PASSWORD FOR "admin" = 'newpassword'
   ```

   如果你想修改其他用户的密码：

   ```sql
   SET PASSWORD FOR "iottest" = 'newpassword'
   ```

4. **重新启用认证**： 编辑配置文件，将 `auth-enabled` 设置为 `true`，然后重启InfluxDB服务：

   ```conf
   [http]
     auth-enabled = true
   ```

   ```bash
   service influxdb restart
   ```

# 2. 数据库操作

## 2.1. 创建数据库

```sql
CREATE DATABASE mydb
```

查看所有数据库：

```sql
SHOW DATABASES
```

使用数据库

```sql
use mydb
```



## 2.2. 插入数据

在InfluxDB中，数据是以时间序列的形式存储的，每条数据称为一个点（point）。一个点包含以下几个部分：

- measurement（测量）：相当于数据库中的表。
- tags（标签）：用于存储元数据，类似于数据库中的索引列。
- fields（字段）：用于存储实际的数据。
- timestamp（时间戳）：记录数据的时间。

插入数据的基本语法如下：

```sql
INSERT measurement,tag_key=tag_value field_key=field_value timestamp
```

例如，插入一条温度数据：

```sql
INSERT temperature,location=office value=23.5 1622547800000000000
```

## 2.3. 查询数据

你可以使用InfluxQL（类似于SQL的查询语言）来查询数据。基本查询语法如下：

```sql
SELECT field_key FROM measurement WHERE tag_key='tag_value'
```

例如，查询办公室的温度数据：

```sql
SELECT value FROM temperature WHERE location='office'
```

你可以进一步限制时间范围，例如查询最近的温度数据：

```sql
SELECT value FROM temperature WHERE location='office' AND time > now() - 1h
```

**常用查询语法**

1. **根据条件查询**

   可以使用 `WHERE` 子句根据条件查询数据。例如，查询办公室温度高于24度的数据：

   ```sql
   SELECT * FROM temperature WHERE location = 'office' AND value > 24
   ```

2. **限制返回结果的数量**

   使用 `LIMIT` 子句限制返回结果的数量。例如，只返回最近的5条记录：

   ```sql
   SELECT * FROM temperature LIMIT 5
   ```

3. **按时间范围查询**

   使用 `time` 字段按时间范围查询数据。例如，查询过去1小时的数据：

   ```sql
   SELECT * FROM temperature WHERE time > now() - 1h
   ```

4. **按时间先后顺序排列查询**

   ```sql
   -- 从最早到最新
   SELECT * FROM temperature ORDER BY time ASC
   
   -- 从最新到最早
   SELECT * FROM temperature ORDER BY time DESC
   ```

   5.你可以结合其他查询条件一起使用 `ORDER BY time`，例如限制返回的记录数：

   ```sql
   SELECT * FROM temperature WHERE location = 'office' ORDER BY time ASC LIMIT 10
   ```



## 2.4. 删除数据

如果你需要删除数据，可以使用以下命令：

删除整个测量的数据：

```sql
DROP MEASUREMENT measurement_name
```

删除数据库

```sql
DROP DATABASE database_name
```

## 2.5. 操作MEASUREMENTS(表)

### 2.5.1. 所有表

1. 显示所有表

   ```sql
   SHOW MEASUREMENTS
   ```

2. 显示某个测量（表）的所有字段

   ```sql
     SHOW FIELD KEYS FROM measurement_name
   ```

3. 显示某个测量（表）的所有标签：

   ```sql
   SHOW TAG KEYS FROM measurement_name
   ```

4. 显示标签值

   ```sqlite
   SHOW TAG VALUES FROM measurement_name WITH KEY = tag_key
   ```

### 2.5.2. 表结构

为了确保你了解测量中的字段和标签，可以使用以下命令查看表结构：

```sql
SHOW FIELD KEYS FROM measurement_name
SHOW TAG KEYS FROM measurement_name
```



