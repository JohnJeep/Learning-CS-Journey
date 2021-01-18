<!--
 * @Author: JohnJeep
 * @Date: 2019-08-02 22:17:14
 * @LastEditTime: 2020-12-28 09:13:02
 * @LastEditors: Please set LastEditors
 * @Description:  MySQL基础知识学习，作为一个使用者的角度
-->

<!-- TOC -->

- [1. 数据库（MySQL）](#1-数据库mysql)
  - [1.1. 参考文章](#11-参考文章)
  - [1.2. RDBMS（关系型数据库）](#12-rdbms关系型数据库)
  - [1.3. 语句规范](#13-语句规范)
  - [1.4. 安装操作](#14-安装操作)
  - [1.5. 用户管理](#15-用户管理)
  - [1.6. 密码与登录](#16-密码与登录)
    - [1.6.1. 权限管理](#161-权限管理)
  - [1.7. MySQL配置文件](#17-mysql配置文件)
  - [1.8. 存储架构](#18-存储架构)
    - [1.8.1. MySQL分层结构](#181-mysql分层结构)
    - [1.8.2. MySQL部件](#182-mysql部件)
  - [1.9. 数值类型](#19-数值类型)
  - [1.10. 日期和时间类型](#110-日期和时间类型)
  - [1.11. 字符串](#111-字符串)
  - [1.12. SHOW命令](#112-show命令)
  - [1.13. 数据库操作](#113-数据库操作)
  - [1.14. 数据表操作](#114-数据表操作)
    - [1.14.1. 创建数据表(create)](#1141-创建数据表create)
    - [1.14.2. 查看数据表(show)](#1142-查看数据表show)
    - [1.14.3. 删除数据表(drop)](#1143-删除数据表drop)
    - [1.14.4. 统计数据表](#1144-统计数据表)
    - [1.14.5. 修改数表(alter)](#1145-修改数表alter)
  - [1.15. 数据操作](#115-数据操作)
    - [1.15.1. 插入数据(insert)](#1151-插入数据insert)
    - [1.15.2. 更新表数据(update)](#1152-更新表数据update)
    - [1.15.3. 删除表数据(delete)](#1153-删除表数据delete)
    - [1.15.4. 查询(检索)表数据(select)](#1154-查询检索表数据select)
      - [1.15.4.1. 子查询](#11541-子查询)
      - [1.15.4.2. 连接](#11542-连接)
  - [1.16. function](#116-function)
  - [1.17. 视图(View)](#117-视图view)
    - [1.17.1. 视图创建](#1171-视图创建)
    - [1.17.2. 视图删除](#1172-视图删除)
  - [1.18. 编码问题](#118-编码问题)
  - [1.19. 引擎](#119-引擎)
  - [1.20. table](#120-table)
  - [1.21. index](#121-index)
  - [1.22. lock](#122-lock)
  - [1.23. transaction](#123-transaction)
  - [1.24. 文件](#124-文件)
  - [1.25. 数据的导入与导出](#125-数据的导入与导出)
    - [1.25.1. 导入和导出txt文件](#1251-导入和导出txt文件)

<!-- /TOC -->

# 1. 数据库（MySQL）
## 1.1. 参考文章
- [MySQL安装教程参考文章](https://blog.csdn.net/u013235478/article/details/50623693)
- [MySQL压缩包安装参考](https://www.cnblogs.com/laumians-notes/p/9069498.html)
- [MySQL卸载参考](https://www.cnblogs.com/zimo-jing/p/7931866.html)
- [MySQL 5.1安装和配置过程中遇到的问题](https://www.cnblogs.com/suiy-160428/p/5552463.html)
- 学习网站
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



## 1.2. RDBMS（关系型数据库）
> 一个关系型数据库由一个或数个表格组成

- 冗余：存储两倍数据，冗余降低了性能，但提高了数据的安全性
- 主键：主键是唯一的。一个数据表中只能包含一个主键。使用主键来查询数据
- 外键：用于关联两个表。
- 复合键：复合键（组合键）将多个列作为一个索引键，一般用于复合索引
- 索引：使用索引可快速访问数据库表中的特定信息。索引是对数据库表中一列或多列的值进行排序的一种结构
- 参照完整性: 要求关系中不允许引用不存在的实体。目的是保证数据的一致性
- DDL(Data Definition Languages)语句：数据定义语言，这些语句定义了不同的数据段、数据库、表、列、索引等数据库对象的定义。常用的语句关键字主要包括create、drop、alter等。
- DML(Data Manipulation Language)语句：数据操纵语句，用于添加、删除、更新和查询数据库记录，并检查数据完整性，常用的语句关键字主要包括insert、delete、udpate 和select 等。
- DCL(Data Control Language)语句：数据控制语句，用于控制不同数据段直接的许可和访问级别的语句。这些语句定义了数据库、表、字段、用户的访问权限和安全级别。主要的语句关键字包括grant、revoke 等。


## 1.3. 语句规范
- 关键字与函数名称全部大写
- 数据库名、表名称、字段名称全部小写
- SQL语句默认是以 分号 结尾，SQL不区分大小写
- 每个表保存一个实体信息


## 1.4. 安装操作
- windows下以管理员身份运行cmd，进入到MySQL的bin目录，执行初始化命令：`mysqld --initialize --user=mysql --console`
- 执行命令进行MySQL服务安装：`mysqld –install mysql`
- Windows启动MySQL服务：`net start mysql`
- Windows停止mysql服务：`net stop mysql`
- 卸载MySQL服务：`sc delete MySQL/mysqld -remove`
- 显示当前日期：`select now();`
- 清除命令行buffer： `mysql \c`
- 记录能够按照字段竖着排列:  `\G` 
- 改变MySQL的分隔符(默认为 `;`)：`delimiter 任意的字符`


## 1.5. 用户管理
- 参考
  - [MySQL在Linux下密码默认安全等级问题](https://blog.csdn.net/kuluzs/article/details/51924374)
  - [MySQL修改密码（三种方法示例）](https://www.yiibai.com/mysql/changing-password.html)

## 1.6. 密码与登录
- 登录：`mysql -u root -p`
- 改密码：MySQL提供了各种可用于更改用户密码的语句，包括UPDATE，SET PASSWORD 和 GRANT USAGE语句。
  - 第一种： `ALTER USER 'root'@'localhost' IDENTIFIED WITH mysql_native_password BY '新密码';`
  - 第二种： `SET PASSWORD FOR 'root'@'localhost' = PASSWORD('新密码')`;


### 1.6.1. 权限管理
- 刷新权限：`flush privileges`
- 创建新用户：`create user '你的用户名'@'%' identified with mysql_native_password by '你的密码';`  
- 创建用户后并刷新权限：`GRANT ALL PRIVILEGES ON *.* TO 'root'@'%' IDENTIFIED BY 'root' WITH GRANT OPTION;`
- 查看数据库中存在的用户：`SELECT DISTINCT CONCAT('User: ''',user,'''@''',host,''';') AS query FROM mysql.user;`
- 查看数据的端口：`show global variables like 'port';`
- 显示当前用户：`select user();`
- 显示当前服务器版本：`select version();`


## 1.7. MySQL配置文件
- 数据文件
  > 数据库文件：Linux下默认路径：`/var/lib/mysql`
  - `.frm` 文件：存放表结构
  - `.myd` 文件：存放表数据
  - `.myi` 文件：存放表索引
  - innodb 引擎下 `.ibd` 文件存放索引和表数据；MyISAM引擎下索引和数据的存放是分开的，在不同的文件夹中。
- 二进制日志：log-bin，用于主重复制。 
- 错误日志：log-error，默认是关闭的，记录严重的警告和错误信息，每次启动和关闭的详细信息等。
- 查询日志：log，默认关闭，记录查询的sql语句，如果开启会减低mysql的整体性能，因为记录日志也是需要消耗系统资源的。
  


## 1.8. 存储架构
- 和其它数据库相比，MySQL有点与众不同，它的架构可以在多种不同场景中应用并发挥良好作用。主要体现在存储引擎的架构上。
- Pluggable Storage Engines. 插件式的存储引擎架构将查询处理和其它的系统任务以及数据的存储提取分离。这种架构可以根据业务的需求和实际需求选择合适的存储引擎。


### 1.8.1. MySQL分层结构
1. 连接层
2. 服务层
3. 引擎层
4. 存储层


### 1.8.2. MySQL部件
- Connectors：指的是不同语言中与SQL的交互。
- Management Serveices & Utilities： 系统管理和控制工具
- Connection Pool：连接池
  - 管理缓冲用户连接，线程处理等需要缓存的需求。负责监听对 MySQL Server 的各种请求，接收连接请求，转发所有连接请求到线程管理模块。
  - 每一个连接上 MySQL Server 的客户端请求都会被分配（或创建）一个连接线程为其单独服务。而连接线程的主要工作就是负责 MySQL Server 与客户端的通信。接受客户端的命令请求，传递 Server 端的结果信息等。线程管理模块则负责管理维护这些连接线程。包括线程的创建，线程的 cache 等。
- SQL Interface：SQL接口。接受用户的SQL命令，并且返回用户需要查询的结果。比如select from就是调用SQL Interface。
- Parser：解析器
  - QL命令传递到解析器的时候会被解析器验证和解析。解析器是由Lex和YACC实现的，是一个很长的脚本。
  - 在 MySQL中我们习惯将所有 Client 端发送给 Server 端的命令都称为 Query，在 MySQL Server 里面，连接线程接收到客户端的一个 Query 后，会直接将该 Query 传递给专门负责将各种 Query 进行分类然后转发给各个对应的处理模块。
  - 解析器的主要功能：
    - 将SQL语句进行语义和语法的分析，分解成数据结构，然后按照不同的操作类型进行分类，然后做出针对性的转发到后续步骤，以后SQL语句的传递和处理就是基于这个结构的。
    - 如果在分解构成中遇到错误，那么就说明这个sql语句是不合理的 

- Optimizer：查询优化器
- Cache和Buffer：查询缓存
- 存储引擎接口







## 1.9. 数值类型
- TINYIN：1字节    范围：-128~127
- SMALLINT: 2字节  范围：-32768~32767
- MEDIUMINT: 3字节
- BIGINT: 8字节
- INT: 4字节
  - int(M): M表示宽度， 常与 zerofill 连用
  > 显示宽度，如果某个数不够定义字段时设置的位数，则前面以0补填，zerofill 属性修改。例如：int(5)   插入一个数'123'，补填后为'00123'
- 浮点数
  - FLOAT: 4字节
  - DOUBLE: 8字节
 - 定点数：用 decimal 表示。



## 1.10. 日期和时间类型
- DATE(3字节)；表示：日期值  YYYY-MM-DD
- TIME(3字节)；表示：时间值或持续时间  HH:MM:SS
- YEAR(1字节)；表示：年份值  YYYY
- DATETIME: 8字节；表示：混合日期和时间值  YYYY-MM-DD HH:MM:SS；要记录年月日时分秒，并且记录的年份比较久远，那么最好使用 DATETIME。
- TIMESTAMP：记录的日期需要让不同时区的用户使用，那么最好使用 TIMESTAMP，因为日期类型中只有它能够和实际时区相对应。
  > 需要经常插入或者更新日期为当前系统时间，则通常使用 TIMESTAMP 来表示。TIMESTAMP 值返回后显示为 “YYYY-MM-DD HH:MM:SS” 格式的字符串，显示宽度固定为 19 个字符。如果想要获得数字值，应在 TIMESTAMP 列添加+0。



## 1.11. 字符串
- CHAR: 固定长度的字符类型，范围：0-255
- VARCHAR: 可变长度的字符类型，范围：0-65535
- TINYBLOB: 0-255  不超过 255 个字符的二进制字符串
- TEXT: 存储更大的文本文件， TEXT 只能保存字符数据。
- BLOB: 也是存储更大的文本文件，BLOB 能用来保存二进制数据，比如照片等。
  > BLOB 和 TEXT 值会引起一些性能问题，特别是在执行了大量的删除操作时。删除操作会在数据表中留下很大的“空洞”，以后填入这些“空洞”的记录在插入的性能上会有影响。为了提高性能，建议定期使用 OPTIMIZE TABLE 功能对这类表进行碎片整理，避免因为“空洞”导致性能问题。

 


> MySQL 5.0 以上的版本：1、一个汉字占多少长度与编码有关：2、UTF－8：一个汉字＝3个字节; 3、GBK：一个汉字＝2个字节

>  CHAR 是固定长度的，所以它的处理速度比 VARCHAR 快得多，但是其缺点是浪费
存储空间，程序需要对行尾空格进行处理，所以对于那些长度变化不大并且对查询速度有较
高要求的数据可以考虑使用 CHAR 类型来存储。在 MySQL 中，不同的存储引擎对 CHAR 和 VARCHAR 的使用原则是不同的。



## 1.12. SHOW命令
- `USE 数据库名`: 选择要操作的Mysql数据库，使用该命令后所有Mysql命令都只针对该数据库操作。
- `SHOW DATABASES`:  列出 MySQL 数据库管理系统的所有数据库列表。
- `SHOW TABLES`:  显示指定数据库的所有表。
- `SHOW GRANTS`： 显示授予用户（所有用户或特定用户）的安全权限。
- `SHOW COLUMNS FROM 数据表`:  显示数据表的属性，属性类型，主键信息 ，是否为 NULL，默认值等其他信息，即查看表的结构。
- `SHOW INDEX FROM 数据表`:  显示数据表的详细索引信息，包括 PRIMARY KEY（主键）


## 1.13. 数据库操作
- 三种方式查看当前数据库
  - `select database();`
  - `status;`
  -  `select tables;`
- 显示当前时间、用户名、数据库版本: `select now(), user(), version();`
- 创建库:  `create database[ if not exists] 数据库名 数据库选项`
  - 数据库选项：
    - CHARACTER SET charset_name
    - COLLATE collation_name
- 查看已有库:  `show databases[ like 'pattern']`
- 查看当前库信息:  `show create database 数据库名`
- 修改库的选项信息:  `alter database 库名 选项信息`
- 删除库(同时删除该数据库相关的目录及其目录内容): `drop database[ if exists] 数据库名`       


## 1.14. 数据表操作
### 1.14.1. 创建数据表(create)
- 创建数据表 `CREATE TABLE [temporary] [IF NOT EXISTS] table_name(column_name data_type, ...);`
  - table_name可以为`[库名].表名`
  - 每个字段必须有数据类型
    - 字段的定义：
      - 字段名 
      - 数据类型： `[NOT NULL | NULL] [DEFAULT default_value] [AUTO_INCREMENT] [UNIQUE [KEY] | [PRIMARY] KEY] [COMMENT 'string']`
  - 最后一个字段后不能有逗号
  - `temporary`为临时表，会话结束时表自动消失  


### 1.14.2. 查看数据表(show)
- 查看数据表列表 `SHOW TABLES [FROM db_name]` 使用FROM db_name可以查看该数据库中所有的表
- 查看数据表结构 
  - `SHOW COLUMNS FROM tab_nmae`
  - `DESC tab_name`
  - `DESCRIBE tab_name`
  - `EXPLAIN tab_name`
- 查看表详细信息：`SHOW CREATE TABLE 表名`
- 获取表信息：`SHOW TABLE STATUS [FROM db_name] [LIKE 'pattern']`
- 查看记录（数据表内容） `SELECT expr, ... FROM tabl_name`


### 1.14.3. 删除数据表(drop)
- 删除数据表：`DROP TABLE table_name;`
- 清空数据表： `TRUNCATE [TABLE] tab_name`
- 复制表结构: `CREATE TABLE 新表名称 LIKE 要复制的表名`
- 复制表结构和数据: `CREATE TABLE 新表名称 [AS] SELECT * FROM 要复制的表名` 
- 检查表是否有错误： `CHECK TABLE tbl_name [, tbl_name] ... [option] ...`
- 优化表: `OPTIMIZE [LOCAL | NO_WRITE_TO_BINLOG] TABLE tbl_name [, tbl_name] ...`
- 修复表:  `REPAIR [LOCAL | NO_WRITE_TO_BINLOG] TABLE tbl_name [, tbl_name] ... [QUICK] [EXTENDED] [USE_FRM]`
- 分析表: `ANALYZE [LOCAL | NO_WRITE_TO_BINLOG] TABLE tbl_name [, tbl_name] ...`


### 1.14.4. 统计数据表
- 查询数据采用SELECT命令
  - 查询数据表中不重复的行：`SELECT DISTINCT col_name FROM tab_name`
  - 从行 5 开始曲 4 行：`SELECT col_name FROM tab_name LIMIT 5, 4`
- 统计某个数据库中有多少张表：`SELECT count(*) TABLES, table_schema FROM information_schema.TABLES where table_schema = '数据库名' GROUP BY table_schema;`


### 1.14.5. 修改数表(alter)
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


## 1.15. 数据操作
### 1.15.1. 插入数据(insert)
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


### 1.15.2. 更新表数据(update)
- 单表更新记录: `UPDATE tab_name SET 字段名=新值1 [, 字段名=新值N]... WHERE [where_condition]`  
  - 若省略`WHERE`语句，表里面的内容都要更新
  - 组成
    - 列名和它们的新值
    - 要更新的表
    - 确定要更新行的过滤条件
    
- 清空某列的数据值：`update tab_name set 字段名=null where 执行的条件` 

- 多表更新: `update emp_1, emp_2 set emp_1.sal = emp_1.sal * emp_2.deptno, emp_2.deptname = emp_1.name where emp_1.deptno=emp_2.deptno`    更新表emp_1与表emp_2的deptno列相等的值，使表emp_1 sal列的值为emp_1.sal * emp_2.deptno, 表emp_2 列deptname的值为 emp_1.name
  > 常用于根据一个表的字段来动态的更新另外一个表的字段属性


### 1.15.3. 删除表数据(delete)
  - 删除单表数据(对行操作): `DELETE FROM tab_name WHERE 删除条件 `    
    - 多行(删除age=-127和age=127的两行)：`delete from info where age in (-128, 127);`
  - 删除多表数据(对行操作): `DELETE FROM tab1_name, tab2_name, ... tabn_name WHERE 删除条件 `
  - 删除原来的表并重新创建一个表，而不是逐行删除表中的数据: `TRUNCATE`


- 删除表信息的方式有两种
  - truncate table table_name;
  - delete * from table_name;
  > 注 : truncate操作中的table可以省略，delete操作中的*可以省略


- truncate、delete 清空表数据的区别
  - truncate 是整体删除 (速度较快)，delete是逐条删除 (速度较慢)
  - truncate 不写服务器 log，delete 写服务器 log，也就是 truncate 效率比 delete高的原因
  - truncate 不激活trigger (触发器)，但是会重置Identity (标识列、自增字段)，相当于自增列会被置为初始值，又重新从1开始记录，而不是接着原来的 ID数。而 delete 删除以后，identity 依旧是接着被删除的最近的那一条记录ID加1后进行记录。如果只需删除表中的部分记录，只能使用 DELETE语句配合 where条件


### 1.15.4. 查询(检索)表数据(select)
- 查看表中所有数据: `SELECT * FROM tab_name WHERE [CONDITION]`
- 去掉重复数据显示使用关键字 `distinct`
  > `SELECT DISTINCT (co_name1, col_name2, ..., col_nameN) FROM tab_name WHERE [CONDITION]`


- `where` 语句对数据表中的数据进行条件筛选，能与运算符组合后进行高级筛选行
  - 在聚合前就对记录进行过滤筛选
  - 采用`AND`、 `OR`关键字; 注意：在计算机中AND的优先级高于OR运算
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


- where、group by、having、order by、limit这些关键字查询的使用顺序？
  > 使用的顺序：where---group by---having---order by---limit


#### 1.15.4.1. 子查询
- where型子查询: 内层查询的结果作为外层的条件
- from型子查询：内层SQL查询的结果当成一张临时表，供外层的SQL再次查询。
- exist型子查询：把外层SQL查询的结果拿到内层SQL中去测试，如果内层SQL成立，则该行取出。
- <font color=red> NULL说用 </font>  
  > null是一种类型，比较时只能用 `is null` 或 `is not null`;碰见运算符时，一律返回为null。使用null效率不高，影响索引的效果


#### 1.15.4.2. 连接
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




## 1.16. function
- `cancat()函数`: 拼接两个列:
- `RTrim()函数`: 去掉值右边的所有空格
- `LTrim()函数`: 去掉值左边的所有空格
- `Upper()`: 将文本转换为大写


## 1.17. 视图(View)
- 什么是视图？
  > 可以看成一张虚拟的表，是表通过某种运算得到的一个投影。


### 1.17.1. 视图创建
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


### 1.17.2. 视图删除
- ` drop view view_name` 删除视图时，只能删除视图的定义，不会删除数据，删除一个或多个视图时，视图中间使用逗号分隔开




## 1.18. 编码问题
- 字符集: `CHARSET = charset_name`
-  查看数据库的编码方式：`show variables like '%char%';`

- 四个概念
  - client(客户端)
  - connection(连接器)
  - server(服务器)
  - result(返回客户端的结果)


- 解决不想乱码
  - 正确选择客户端的编码
  - 合理选择连接器的编码
  - 正确选择返回内容的编码
  -  如果client、connection、result三者的编码都为GBK，则可以简写为：`set names gbk;`


## 1.19. 引擎
- 存储引擎: `ENGINE = engine_name`
  - 表在管理数据时采用的不同的数据结构，结构不同会导致处理方式、提供的特性操作等不同
- 常见的引擎：InnoDB, MyISAM, Memory/Heap, BDB, Merge, Example, CSV, MaxDB, Archive
- 不同的引擎在保存表的结构和数据时，采用不同的方式
  - MyISAM表文件含义：.frm表定义，.MYD表数据，.MYI表索引
  - InnoDB表文件含义：.frm表定义，表空间数据和日志文件
- 查看mysql支持什么引擎信息： `SHOW ENGINES`
- 查看mysql当前默认的存储引擎：`  show variables like '%storage_engine%';`
- 查看某个表用了什么引擎：` show create table 表名;`
- 显示存储引擎的日志或状态信息: `SHOW ENGINE 引擎名 {LOGS|STATUS}`
- 修改表引擎方法: ` alter table table_name engine=InnoDB; `


## 1.20. table
- 在InnoDB引擎中，表都是根据主键顺序组织存放的，这种存储方式称为索引组织表(index organized table)。


## 1.21. index
- Innodb引擎支持下面的几种引擎？
  - B+树：是关系型数据库职工查找最为常用和最有效的索引。
  - 全文索引
  - 哈希 



## 1.22. lock
- 什么是latch？
  > latch一般叫为闩锁（轻量级的锁）。在innodb 引擎中，latch有分为mutex 和rwlock。目的是：用来保证并发线程操作临界资源的正确性，并且通常没有死锁检测的机制。

- 什么是lock？
  > 锁是数据库系统中区别于文件系统的一个关键特性。

- 为什么要有lock?
  > 用于管理对共享资源的并发问题，提供数据的完整性和一致性。

- lock的对象是事务，用来锁定数据库中的对象，如tables，pages，rows，一般的lock的对象仅在事务commit和rollback后进行释放，lock是有死锁机制的。


- 命令
  - `show engine innodb status \G;`  查看当前数据库中锁的请求。


## 1.23. transaction
- 参考：
  - [数据库事务的四大特性以及事务的隔离级别](https://blog.csdn.net/FX677588/article/details/76747864)
  
- 什么叫 transaction？
  > 一个SQL语句或者一组复杂的SQL语句就是一个事务（transaction）。事务时访问并更新数据库中各数据项的一个程序执行单元。
  
- 为什么要有 transaction？
  > 事务会把数据库从一种一致状态转换为另一种一致状态，在数据库提交工作时，可以确保要么所有修改都已经保存了，要么所有修改都不保存。


- 事务的四大特性（ACID） 
  - 原子性（Atomicity）：用于保证数据的一致性，由一组相关的DML语句组成，修改的操作要么全部成功，要么全部失败。（失败的时候要进行回滚操作[rollback]）
  - 一致性（Consistency）：一个事务执行之前和执行之后都必须处于一致性状态，即操作前后值得变化，逻辑上保持一致。
  - 隔离性（Isolation）：如果多个事务同时并发执行，但每个事务会各自独立执行，不能被其他事务的操作所干扰。
  - 持久性（Durability）：事务一旦完成，无法撤销。一个事务一旦被提交了，对数据库中的数据的改变就是永久性的，即便是在数据库系统遇到故障的情况下也不会丢失提交事务的操作。通过 `redo log(重做日志)` 和 `undo log` 操作来保证事务的一致性。
    - redo: 通常是物理日志，记录页的物理修改操作；redo是恢复提交事务修改的页操作。
    - undo：通常是逻辑日志，根据每行记录来进行记录；undo是回滚行记录到某个特定的版本。
   

- 事务的操作
  - `start transaction`：显式开启事务  
  - `commit`：提交事务
  - `rollback`：回滚用户的事务，撤销正在进行的所有未提交的修改。
  - `savepoint identifier`：允许在事务中创建一个保存点，一个事务中可以有多个 savepoint
  - `release savepoint identifier`：删除一个事务的保存点，当没有一个保存点执行这条语句时，会抛出一个异常。
  - `rollback to [savepoint] identifier`：把事务回滚到某个标记点。
  - `set transaction`：设置事务的隔离级别


## 1.24. 文件
- 数据文件目录: `DATA DIRECTORY = '目录'`
- 索引文件目录: `INDEX DIRECTORY = '目录'`
- 表注释: `COMMENT = 'string'`
- 分区选项: `PARTITION BY ...` (详细见手册)


## 1.25. 数据的导入与导出
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


### 1.25.1. 导入和导出txt文件
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


