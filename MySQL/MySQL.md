```
Description: MySQL基础知识学习，作为一个使用者的角度
Author: JohnJeep
Date: 2019-08-02 22:17:14
LastEditTime: 2019-09-15 19:22:15
LastEditors: JohnJeep
```

 参考文章
- [MySQL安装教程参考文章](https://blog.csdn.net/u013235478/article/details/50623693)
- [MySQL压缩包安装参考](https://www.cnblogs.com/laumians-notes/p/9069498.html)
- [MySQL卸载参考](https://www.cnblogs.com/zimo-jing/p/7931866.html)
- [MySQL 5.1安装和配置过程中遇到的问题](https://www.cnblogs.com/suiy-160428/p/5552463.html)


学习网站和博客
- 网站
  - [MySQL Tutorial 英文档](http://www.mysqltutorial.org/)
  - [国外的Planet MySQL](https://planet.mysql.com/)
  - [W3school SQL教程](https://www.w3school.com.cn/sql/sql_syntax.asp)


- 博客
  - [万乐荣MySQL讲解](http://www.notedeep.com/note/38/page/328)
  - [何登成的技术博客](http://hedengcheng.com/)
  - [orczhou博客](http://www.orczhou.com/)
  - [追风刀·丁奇 - ITeye技术网站](https://dinglin.iteye.com/)
  - [阿里云数据库高级专家彭立勋](http://www.penglixun.com/)
  - [国外 Percona's MySQL & InnoDB performance and scalability blog](https://www.percona.com/blog/)
  - [比较详细的MySQL学习笔记](https://shockerli.net/post/1000-line-mysql-note/)



# RDBMS（关系型数据库）
> 一个关系型数据库由一个或数个表格组成

- 冗余：存储两倍数据，冗余降低了性能，但提高了数据的安全性
- 主键：主键是唯一的。一个数据表中只能包含一个主键。使用主键来查询数据
- 外键：用于关联两个表。
- 复合键：复合键（组合键）将多个列作为一个索引键，一般用于复合索引
- 索引：使用索引可快速访问数据库表中的特定信息。索引是对数据库表中一列或多列的值进行排序的一种结构
- 参照完整性: 要求关系中不允许引用不存在的实体。目的是保证数据的一致性
- DDL（Data Definition Languages）语句：数据定义语言，这些语句定义了不同的数据段、数据库、表、列、索引等数据库对象的定义。常用的语句关键字主要包括create、drop、alter等。
- DML（Data Manipulation Language）语句：数据操纵语句，用于添加、删除、更新和查询数据库记录，并检查数据完整性，常用的语句关键字主要包括insert、delete、udpate 和select 等。
- DCL（Data Control Language）语句：数据控制语句，用于控制不同数据段直接的许可和访问级别的语句。这些语句定义了数据库、表、字段、用户的访问权限和安全级别。主要的语句关键字包括grant、revoke 等。

## 语句规范
- 关键字与函数名称全部大写
- 数据库名、表名称、字段名称全部小写
- SQL语句默认是以分号结尾,SQL是不区分大小写
- 每个表保存一个实体信息


## 安装操作
- 初始化：以管理员身份运行cmd，进入到MySQL的bin目录，执行初始化命令：
`mysqld --initialize --user=mysql --console`
- 执行命令进行MySQL服务安装：`mysqld –install mysql`
- 启动MySQL服务：`net start mysql`
- 停止mysql服务：`net stop mysql`
- 登录 `mysql -uroot -p`
- 改密码`ALTER USER 'root'@'localhost' IDENTIFIED WITH mysql_native_password BY '新密码';`
- 卸载MySQL服务：`sc delete MySQL/mysqld -remove`
- 显示当前服务器版本`select version();`
- 显示当前日期`select now();`
- 显示当前用户`select user();`
- 改变mysql的结束语句：`DELIMITER 任意的字符 ` 
- 清除命令行buffer `mysql \c`
- 记录能够按照字段竖着排列:  `\G` 
- 改变MySQL的分隔符(默认为 `;`)：`delimiter 任意的字符`



### 数值类型
- TINYIN：1字节    范围：-128~127
- SMALLINT: 2字节  范围：-32768~32767
- MEDIUMINT: 3字节
- INT: 4字节
  - int(M): M表示宽度， 常与zerofill连用
  > 显示宽度，如果某个数不够定义字段时设置的位数，则前面以0补填，zerofill 属性修改
        例：int(5)   插入一个数'123'，补填后为'00123'
- BIGINT: 8字节
- FLOAT: 4字节
- DOUBLE: 8字节


### 日期和时间类型
- DATE: 3字节  日期值  YYYY-MM-DD
- TIME: 3字节  时间值或持续时间  HH:MM:SS
- YEAR: 1字节  年份值  YYYY
- DATETIME: 8字节  混合日期和时间值  YYYY-MM-DD HH:MM:SS


### 字符串
- CHAR: 0-255
- VARCHAR: 0-65535
- TINYBLOB: 0-255  不超过 255 个字符的二进制字符串
> MySQL 5.0 以上的版本：1、一个汉字占多少长度与编码有关：2、UTF－8：一个汉字＝3个字节; 3、GBK：一个汉字＝2个字节


### SHOW命令
- USE 数据库名: 选择要操作的Mysql数据库，使用该命令后所有Mysql命令都只针对该数据库
- SHOW DATABASES:  列出 MySQL 数据库管理系统的数据库列表
- SHOW TABLES:  显示指定数据库的所有表
- SHOW GRANTS： 显示授予用户（所有用户或特定用户）的安全权限
- SHOW COLUMNS FROM 数据表:  显示数据表的属性，属性类型，主键信息 ，是否为 NULL，默认值等其他信息
- SHOW INDEX FROM 数据表:  显示数据表的详细索引信息，包括PRIMARY KEY（主键）


### 数据库操作
- 查看当前数据库： `select database();`
- 显示当前时间、用户名、数据库版本: `select now(), user(), version();`
- 创建库:  `create database[ if not exists] 数据库名 数据库选项`
  - 数据库选项：
    - CHARACTER SET charset_name
    - COLLATE collation_name
- 查看已有库:  `show databases[ like 'pattern']`
- 查看当前库信息:  `show create database 数据库名`
- 修改库的选项信息:  `alter database 库名 选项信息`
- 删除库(同时删除该数据库相关的目录及其目录内容): `drop database[ if exists] 数据库名`       





### 数据表操作

#### 创建数据表(create)
- 创建数据表 `CREATE TABLE [temporary] [IF NOT EXISTS] table_name(column_name data_type, ...);`
  - table_name可以为`[库名].表名`
  - 每个字段必须有数据类型
    - 字段的定义：
      - 字段名 
      - 数据类型： `[NOT NULL | NULL] [DEFAULT default_value] [AUTO_INCREMENT] [UNIQUE [KEY] | [PRIMARY] KEY] [COMMENT 'string']`
  - 最后一个字段后不能有逗号
  - `temporary`为临时表，会话结束时表自动消失  


#### 查看数据表(show)
- 查看数据表列表 `SHOW TABLES [FROM db_name]` 使用FROM db_name可以查看该数据库中所有的表
- 查看数据表结构 
  - `SHOW COLUMNS FROM tab_nmae`
  - `DESC tab_name`
  - `DESCRIBE tab_name`
  - `EXPLAIN tab_name`
- 查看表详细信息：`SHOW CREATE TABLE 表名`
- 获取表信息：`SHOW TABLE STATUS [FROM db_name] [LIKE 'pattern']`
- 查看记录（数据表内容） `SELECT expr, ... FROM tabl_name`


#### 删除数据表(drop)
- 删除数据表：`DROP TABLE table_name;`
- 清空数据表： `TRUNCATE [TABLE] tab_name`
- 复制表结构: `CREATE TABLE 新表名称 LIKE 要复制的表名`
- 复制表结构和数据: `CREATE TABLE 新表名称 [AS] SELECT * FROM 要复制的表名` 
- 检查表是否有错误： `CHECK TABLE tbl_name [, tbl_name] ... [option] ...`
- 优化表: `OPTIMIZE [LOCAL | NO_WRITE_TO_BINLOG] TABLE tbl_name [, tbl_name] ...`
- 修复表:  `REPAIR [LOCAL | NO_WRITE_TO_BINLOG] TABLE tbl_name [, tbl_name] ... [QUICK] [EXTENDED] [USE_FRM]`
- 分析表: `ANALYZE [LOCAL | NO_WRITE_TO_BINLOG] TABLE tbl_name [, tbl_name] ...`


#### 统计数据表
- 查询数据采用SELECT命令
  - 查询数据表中不重复的行：`SELECT DISTINCT col_name FROM tab_name`
  - 从行 5 开始曲 4 行：`SELECT col_name FROM tab_name LIMIT 5, 4`


- 统计某个数据库中有多少张表：`SELECT count(*) TABLES, table_schema FROM information_schema.TABLES where table_schema = '数据库名' GROUP BY table_schema;`


#### 修改数表(alter)
- 修改数据表
  - 对表进行重命名
    - `RENAME TABLE 原表名 TO 新表名`
    - `RENAME TABLE 原表名 TO 库名.表名`    （可将表移动到另一个数据库）
    - 尽量少去更改表和列的名称，在某些情况下可能导致一些视图不能使用


  - 修改表的字段结构：` ALTER TABLE 表名 操作名`
    - 添加单列 `ALTER TABLE tab_name ADD [COLUMN] column_name column_definition[FIRST | AFTER col_name]`
    - 添加多列 `ALTER TABLE tab_name ADD [COLUMN] (col_name col_definition, ...)` 不能指定位置关系，只能在列的最下方
    - 删除列 `ALTER TABLE tab_name DROP [COLUMN] col_name_1,DROP [COLUMN] col_name_2`

      - 增加主键：`ALTER TABLE tab_name ADD [CONSRTAINT [symbol]] PRIMARY KEY [index_type](index_col_name)`
      - 删除主键约束(删除主键前需删除其AUTO_INCREMENT属性): `ALTER TABLE tab_name DROP PRIMARY KEY`
      - 创建唯一索引: `ALTER TABLE tab_name ADD UNIQUE [索引名] (字段名)`
      - 删除唯一索引: `ALTER TABLE tab_name DROP {UNIQUE | KEY} index_name`
      - 创建普通索引: `ALTER TABLE tab_name ADD INDEX [索引名] (字段名) ` 
      - 删除索引: `ALTER TABLE tab_name DROP INDEX 索引名`
      - 添加外键约束 `ALTER TABLE tab_name ADD [CONSTRAINT [symbol]] FOREIGN KEY [index_name] (index_col_name, ...) reference_definition`
        
        > 例如：`ALTER TABLE county ADD FOREIGN KEY(pid) REFERENCES provinces(id);` pid为子键，id为父键
      - 删除外键约束 `ALTER TABLE tab_name DROP FOREIGN KEY 约束名称`
- 添加和删除默认约束 `ALTER TABLE tab_name ALTER [COLUMN] col_name {SET DEFAULT 设置的值 | DROP DEFAULT}`
      
      - 修改列属性和参数 `ALTER TABLE tab_name MODIFY [COLUMN] col_name column_definition [FIRST | AFTER col_name]`
        - 改变列的位置和数据类型，数据类型的改变会导致数据的丢失。
  - 对字段属性进行修改，不能修改字段名(所有原有属性也需写上)。
    
      - 对字段名修改: `ALTER TABLE tab_name CHANGE[ COLUMN] 原字段名 新字段名 新字段属性 新参数`


### 数据操作

#### 插入数据(insert)
  - 单条插入数据：`INSERT INTO table_name ( field1, field2,...fieldN )
                        VALUES
                        ( value1, value2,...valueN );`
    - 如果省略字段列表，要插入数据的值与字段顺序一致，不能任意调整位置
    - 可同时插入多条数据记录：`INSERT INTO table_name ( field1, field2,...fieldN )
                        VALUES
                        ( value1, value2,...valueN ),
                        ( value1, value2,...valueN ),
                        ( value1, value2,...valueN ),`
  - 多条数据的插入，节省了网络开销、提高了插入效率


#### 更新表数据(update)
  - 单表更新记录: `UPDATE tab_name SET 字段名=新值1 [, 字段名=新值N]... WHERE [where_condition]`  
    - 若省略`WHERE`语句，表里面的内容都要更新
    - 组成
      - 列名和它们的新值
      - 要更新的表
      - 确定要更新行的过滤条件
    

    - 清空某列的数据值：`update tab_name set 字段名=null where 执行的条件` 


  - 多表更新: `update emp_1, emp_2 set emp_1.sal = emp_1.sal * emp_2.deptno, emp_2.deptname = emp_1.name where emp_1.deptno=emp_2.deptno`    更新表emp_1与表emp_2的deptno列相等的值，使表emp_1 sal列的值为emp_1.sal * emp_2.deptno, 表emp_2 列deptname的值为 emp_1.name
  > 常用于根据一个表的字段来动态的更新另外一个表的字段属性


#### 删除表数据(delete)
  - 删除单表数据(对行操作): `DELETE FROM tab_name WHERE 删除条件 `    
    - 多行(删除age=-127和age=127的两行)：`delete from info where age in (-128, 127);`
  - 删除多表数据(对行操作): `DELETE FROM tab1_name, tab2_name, ... tabn_name WHERE 删除条件 `
  - 删除原来的表并重新创建一个表，而不是逐行删除表中的数据: `TRUNCATE`


删除表信息的方式有两种 :
truncate table table_name;
delete * from table_name;
注 : truncate操作中的table可以省略，delete操作中的*可以省略

truncate、delete 清空表数据的区别 :
1> truncate 是整体删除 (速度较快)，delete是逐条删除 (速度较慢)
2> truncate 不写服务器 log，delete 写服务器 log，也就是 truncate 效率比 delete高的原因
3> truncate 不激活trigger (触发器)，但是会重置Identity (标识列、自增字段)，相当于自增列会被置为初始值，又重新从1开始记录，而不是接着原来的 ID数。而 delete 删除以后，identity 依旧是接着被删除的最近的那一条记录ID加1后进行记录。如果只需删除表中的部分记录，只能使用 DELETE语句配合 where条件




#### 查询(检索)表数据(select)
  - 查看表中所有数据: `SELECT * FROM tab_name WHERE [CONDITION]`
  - 去掉重复数据显示：关键字`distinct`: `SELECT DISTINCT (co_name1, col_name2, ..., col_nameN) FROM tab_name WHERE [CONDITION]`


  - `where` 语句对数据表中的数据进行条件筛选，且与运算符组合进行高级筛选行
    - 在聚合前就对记录进行过滤筛选
    - 采用`AND`、 `OR`关键字; 在计算机中AND的优先级高于OR运算
    - 关键字`IN`: 对指定范围的内容进行匹配，与`OR`等效；
    - 关键字`NOT`: 对IN 、BETWEEN 和EXISTS子句取反
    - like , not like ('%'匹配任何字符出现任意次数,'_'匹配任意单个字符) `SELECT col_name FROM tab_name WHERE tab_name LIKE 'char%' `
    - 检查是否为空值(is null或is not null): `SELECT col_name1, col_name2, ..., col_nameN FROM tab_name WHERE [CONDITION] IS NULL`


  - 分组`(group by)`：表示要进行分类聚合的字段，通常与聚合函数一起使用
    - 分组列中具有NULL值，则NULL将作为一个分组返回。如果列中有多行NULL值，它们将分为一组。 
    - GROUP BY子句必须出现在WHERE子句之后，ORDER BY子句之前。
    - GROUP BY子句可以包含任意数目的列
    - `WITH ROLLUP`: 是否对分类聚合后的结果进行再汇总
    - 聚合函数：
      - AVG(): 求某列的平均值；忽略了列值为NULL的行
      - COUNT(): 返回某列的行数
        - COUNT(*)：对表中行的数目进行计数，不管表列中包含的是空值（NULL）还是非空值
        - COUNT(column)：对指定列并有值的行进行计数，得到行的总数，忽略NULL值。
      - MAX(): 求某列的最大值
      - MIN(): 求某列的最小值
      - SUN(): 对某列值之和 `SELECT SUN(column) FROM tab_name;`
> <font color=red> 注意：select 选择的列(a,b,c)必须在group by 分组的列名(a, b, c, d, e)中，否则会报错。但是select中可以包含聚合函数和别名</font> 


  - `HAVING`: 对聚合分类后的<font color=red>结果集</font>合再进行条件的筛选，过滤分组
    - 与 where 功能、用法相同，执行时机不同。
    - where 在开始时执行检测数据，对原数据进行过滤。
    - having 对筛选出的结果再次进行过滤。
    - having 字段必须是查询出来的，where 字段必须是数据表存在的。
    - where 不可以使用字段的别名，having 可以。因为执行 where代码时，可能尚未确定列值。
    - where 不可以使用聚合函数。一般使用聚合函数才会用 having
    - SQL标准要求 `HAVING` 必须引用 `GROUP BY` 子句中的列或用聚合函数中的列。


  - ` order by`: 对最终结果集进行排序，在where/group by/having等之后，顺序不能乱
    - `DESC`:降序排序;  `ASC`: 升序排序
    - `order by 列名1 desc, 列名2 asc;`    多个排序之间使用` , `号隔开

  - `limit` 限制条件
    - `limit [offset, N]`  
      - offset(偏移量): 跳过offset行；
      - N：实际取得N条数据
  > 使用的顺序：where---group by---having---order by---limit


##### 子查询
- where型子查询: 内层查询的结果作为外层的条件
- from型子查询：内层SQL查询的结果当成一张临时表，供外层的SQL再次查询。
- exist型子查询：把外层SQL查询的结果拿到内层SQL中去测试，如果内层SQL成立，则该行取出。


- <font color=red> NULL说用 </font>
  
  > null是一种类型，比较时只能用 `is null` 或 `is not null`;碰见运算符时，一律返回为null。使用null效率不高，影响索引的效果


##### 连接
- 使用两表相乘生成第三个表的方法，再进行查询。这样做的方法效率很低，对内存的开销很大。
- mysql中不支持外链接
- 左连接: `A left join B on 条件`:
  - 若条件为真，则B表对应的行取出
  - 可以查询A、B表中所有的列
  - 可以把 `A left join B on 条件`看成一个新的表C
- 右链接:`B right join A on 条件`:
  - 与左连接 `A left join B on 条件`等价
  - 在mysql中，尽量使用左连接，具有很好的移植性
- 内连接(inner): 是左右连接的交集


- 联合(union): 合并2条或多条语句的结果集
  - 可以多张表操作 
  - 多张表的列名称不一致时也可以用，以第一张表的 字段名为基石。
  - 只要结果集中列的数量一致就可以。
  - 内层的order by 语句单独使用，不会影响结果集，在执行期间被mysql的代码分析器优化掉了。
  - 若果order by与limit配合使用，会影响返回的结果集，有作用。
  - 使用union后返回的结果集有重复，默认会去除结果集重复的数据；若果不想去除重复的数据，使用 ` union all` 关键字




### 函数
- `cancat()函数`: 拼接两个列:
- `RTrim()函数`: 去掉值右边的所有空格
- `LTrim()函数`: 去掉值左边的所有空格
- `Upper()`: 将文本转换为大写




### 视图(View)
> 可以看成一张虚拟的表，是表通过某种运算得到的一个投影。

#### 创建
- 创建视图` create view 视图名 as select语句`
- 作用：
  - 简化查询：统计复杂的结果时，先用视图生成一个中间结果，再查询视图。
  - 精确的权限控制。
  - 分表时使用
- 表的变化直接影响视图的变化
- 视图压根不能改。
- 视图在某些条件下也可以改变(drop、delete、alter)，必须是视图的数据与表的数据呈现一一对应才可以改变。若使用`order by ` 和`limit`之后，表里面的数据域视图里的数据不是一一对应关系。
- `algorithm`用法
  - 有三个参数组成
    - merge：条件合并。（简单的视图，并没有建立临时表，而是把条件存储起来，下次查询时再把条件合并，然后再去查表）
    - undefined：未定义，由系统决定采用创建临时表还是简单的组合语句。
    - temptable：创建临时表


#### 删除
- ` drop view view_name` 删除视图时，只能删除视图的定义，不会删除数据，删除一个或多个视图时，视图中间使用逗号分隔开




### 编码问题
- 字符集: `CHARSET = charset_name`
- 四个概念
  - client(客户端)
  - connection(连接器)
  - server(服务器)
  - result(返回客户端的结果)
- 如果client、connection、result三者的编码都为GBK，则可以简写为：`set names gbk;`


- 解决不想乱码
  - 正确选择客户端的编码
  - 合理选择连接器的编码
  - 正确选择返回内容的编码


### 引擎
- 存储引擎: `ENGINE = engine_name`
  - 表在管理数据时采用的不同的数据结构，结构不同会导致处理方式、提供的特性操作等不同
  - 常见的引擎：InnoDB MyISAM Memory/Heap BDB Merge Example CSV MaxDB Archive
  - 不同的引擎在保存表的结构和数据时，采用不同的方式
  - MyISAM表文件含义：.frm表定义，.MYD表数据，.MYI表索引
  - InnoDB表文件含义：.frm表定义，表空间数据和日志文件


  - 查看mysql支持什么引擎信息： `SHOW ENGINES`
  - 查看mysql当前默认的存储引擎：`  show variables like '%storage_engine%';`
  - 查看某个表用了什么引擎：` show create table 表名;`
  - 显示存储引擎的日志或状态信息: `SHOW ENGINE 引擎名 {LOGS|STATUS}`
  - 修改表引擎方法: ` alter table table_name engine=InnoDB; `



- 事务四个特性

参考：[数据库事务的四大特性以及事务的隔离级别](https://blog.csdn.net/FX677588/article/details/76747864)
  - 概念:每一个SQL语句就是一个事务。`start transaction`：开启事务  `commit`：提交 


  - 原子性（Atomicity）：用于保证数据的一致性，由一组相关的DML语句组成，改组的语句要么全部成功，要么全部失败。（失败的时候要进行回滚操作[rollback]）
  - 一致性（Consistency）：一个事务执行之前和执行之后都必须处于一致性状态，即操作前后值得变化，逻辑上保持一致。
  - 隔离性（Isolation）：如果多个事务同时并发执行，但每个事务会各自独立执行，不能被其他事务的操作所干扰。
  - 持久性（Durability）：事务一旦完成，无法撤销。一个事务一旦被提交了，对数据库中的数据的改变就是永久性的，即便是在数据库系统遇到故障的情况下也不会丢失提交事务的操作


- 数据文件目录: `DATA DIRECTORY = '目录'`
- 索引文件目录: `INDEX DIRECTORY = '目录'`
- 表注释: `COMMENT = 'string'`
- 分区选项: `PARTITION BY ...` (详细见手册)




### 数据的导入与导出
[Mysql备份还原数据库之mysqldump实例及参数详细说明](https://www.cnblogs.com/xuejie/archive/2013/01/11/2856911.html)

- `mysqldump`导出固定条件的数据库
  - 导出整个数据库: `mysqldump -u 用户名 -p 数据库名 > 导出的文件名 `
    
    > mysqldump -u wcnc -p smgp_apps_wcnc > wcnc.sql
  - 导出一个表: `mysqldump -u 用户名 -p 数据库名 表名> 导出的文件名`
    
    > mysqldump -u wcnc -p smgp_apps_wcnc users> wcnc_users.sql
  - 导出一个数据库结构: `mysqldump -u wcnc -p -d --add-drop-table smgp_apps_wcnc >d:\wcnc_db.sql` 
  
    > #-d 不导出数据只导出结构 --add-drop-table 在每个create语句之前增加一个drop table  
  - 批量导出多个数据库：`mysqldump  -uroot -p --databases db_1 db_2` 
  
- 使用`source`命令导入数据库
```
  #进入mysql数据库控制台，
  mysql -u root -p 
  mysql>use 数据库
  mysql>set names utf8; （先确认编码，如果不设置可能会出现乱码，注意不是UTF-8） 
  #然后使用source命令，后面参数为脚本文件（如这里用到的.sql）
  mysql>source d:\wcnc_db.sql
```


##### 导入和导出txt文件
- 导入txt文件
```
mysql> load data infile 'd.txt' into table table_name 
    -> CHARACTER SET utf8,              // 设置导入的编码格式，防止乱码，可选
    -> fields terminated by','
    -> lines terminated by'\r\n'
```


- 导出txt文件
```
 mysql> select * from table_name into outfile 'd.txt'
    -> fields terminated by','
    -> lines terminated by'\r\n'
```


- 只导入部分字段的数据
```
 mysql> load data infile 'd.txt' into table tt
    -> lines terminated by'\r\n'
    -> (name);
    // name为字段名，可以为多个
```


