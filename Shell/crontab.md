<!--
 * @Author: JohnJeep
 * @Date: 2023-03-07 11:13:26
 * @LastEditors: JohnJeep
 * @LastEditTime: 2024-11-25 10:00:28
 * @Description: Crontab 定时任务命令用法
 * Copyright (c) 2023 by John Jeep, All Rights Reserved. 
-->

# crontab 用法


## 简介
crontab 命令主要用于设置命令行或者脚本周期性的执行。该命令从标准输入设备读取指令，并将其存放于文件中，以供之后读取和执行。本文主要讲述crontb命令的基本语法和配置方法。


## 用法
`crontab` 是一个用于管理用户的 cron 任务的命令行工具。它允许你创建、编辑、查看和删除 cron 任务。以下是 `crontab` 命令的一些常用用法：

命令格式

```sh
crontab [-u user] -e -l -r
```

- -u 用户名，不加 -u 参数默认为当前用户。
- -e 编辑 crontab 文件。
- -l 列出 crontab 文件中的内容。取值来源为 `/var/spool/cron`下对应的文件
- -r 删除 crontab 文件。

1. 查看当前用户的 cron 任务：
  ```
  crontab -l
  ```

  该命令将显示当前用户的 cron 任务列表。

2. 编辑当前用户的 cron 任务：
  ```
  crontab -e
  ```

  该命令将打开一个文本编辑器，允许你编辑当前用户的 cron 任务。在编辑器中，你可以添加、修改或删除 cron 行。完成编辑后保存并关闭编辑器，cron 任务将被更新。

3. 删除当前用户的 cron 任务：
  ```
  crontab -r
  ```

  该命令将删除当前用户的所有 cron 任务。

4. 从文件导入 cron 任务：
  ```
  crontab <filename>
  ```

  该命令将从指定的文件中导入 cron 任务。文件应包含符合 cron 语法的任务行。导入后，它将替换当前用户的所有 cron 任务。

5. 将 cron 任务导出到文件：
  ```
  crontab -l > <filename>
  ```

  该命令将当前用户的 cron 任务导出到指定的文件。

这些是 `crontab` 命令的一些常见用法。使用这些命令，你可以管理和操作 cron 任务，定期执行你所需要的任务。请注意，使用 `crontab` 命令需要适当的权限，通常仅限于普通用户管理其自己的 cron 任务。

命令说明

```sh
# Example of job definition:
 .---------------- minute (0 - 59)
 |  .------------- hour (0 - 23)
 |  |  .---------- day of month (1 - 31)
 |  |  |  .------- month (1 - 12) OR jan,feb,mar,apr ...
 |  |  |  |  .---- day of week (0 - 6) (Sunday=0 or 7) OR sun,mon,tue,wed,thu,fri,sat
 |  |  |  |  |
 *  *  *  *  * command/script
```

一个定时任务的配置共包括6个字段，分别是分、时、日、月、周、命令行或脚本，每一列取值的范围或者含义如上述格式中注释描述。**特别注意一点是命令行或者脚本一定要配置成绝对路径。**

每个 `*` 代表一个时间字段，顺序如下：

1. **第一个 `\*`：分钟**（取值范围：`0-59`）
2. **第二个 `\*`：小时**（取值范围：`0-23`）
3. **第三个 `\*`：日期**（取值范围：`1-31`）
4. **第四个 `\*`：月份**（取值范围：`1-12`）
5. **第五个 `\*`：星期**（取值范围：`0-7`，0 和 7 都表示星期天）

`command` 是要执行的命令或脚本的路径。

### 示例

- `0 0 * * * command`: 每天午夜 0 点执行一次 `command`。
- `*/5 * * * * command`: 每 5 分钟执行一次 `command`。
- `15 14 * * * command`: 每天下午 2:15 执行一次 `command`。
- `0 22 * * 1-5 command`: 每周一到周五的晚上 10 点执行一次 `command`。
- `0 0 1 * * command`: 每月 1 号的午夜 0 点执行一次 `command`。

### 特殊符号

- `*`：代表任意值。例如在第1列表示每分钟执行，第2列表示每小时执行，第3列表示每天执行。
- `*/n`：表示每隔 `n` 个单位时间。例如 `*/6` 表示每隔 6 小时。
- `,`：用来分隔多个值。例如，`1,15,30` 表示分钟 1、15 和 30 分别执行。
- `-`：指定一个范围。例如，`1-5` 表示从 1 到 5 之间的值。
- `@reboot`：系统启动后立即执行。


## 示例

- 每天晚上10点运行`rumenz.sh`脚本
  ```shell
  0 22 * * * /root/rumenz.sh
  ```

- 每月的1,3,7的早上8.30运行`rumenz.sh`
  ```shell
  30 8 1,3,7 * * /root/rumenz.sh
  ```

- 每周六,日的的凌晨2点执行`rumenz.sh`
  ```shell
  0 2 * * 6,0 /root/rumenz.txt
  ```

- 每天的的18点到23点每30分执行`rumenz.sh`
  ```
  0,30 18-23 * * /root/rumenz.sh
  
  //或者
  */30 18-23 * * /root/rumenz.sh
  ```

- 每天凌晨2点访问一个网址
  ```
  0 2 * * * /usr/bin/curl https://rumenz.com
  ```


# References
- [微信公众号：Linux之crontab使用技巧](https://mp.weixin.qq.com/s?__biz=MzI4MDEwNzAzNg==&mid=2649459914&idx=2&sn=8f7e0735aceea33cf8117d312e1850d9&key=e459974591e3bac8f20384884c0fe290a53a9ac6358e9fc28da14268f503ad46863b05b9a889c0c144f38badd3be015a4f5decec10c4cb18ff4451985f916d5e88fda50d2ba31ac64cd477a6f254f42b136155835c7239e46eaf2bf956de1645793ed89f50ddd70f773eddb64cf2fc34ba664d2b066b9d3f9c0946570f7f74ce&ascene=0&uin=MTE2MDU5MjIzNA%3D%3D&devicetype=Windows+10+x64&version=6309001c&lang=zh_CN&countrycode=CN&exportkey=n_ChQIAhIQ5CCO7T21W4m%2FHwxeQZ3VihLgAQIE97dBBAEAAAAAAD2BFie%2FpmYAAAAOpnltbLcz9gKNyK89dVj0jSgivwYV2B3yIwV6skhUZ%2F21hP7%2FPBzb%2FRAB4wNgG%2BQrwySrMxiEetlGog7JYCky9UKJ55h9c%2B0b94pVa2DwKSj6ft6o842FlbRPuZyIUIG%2FS%2F%2BHE7urRVA3%2FWPK1tfwgpxbHYv2wLhgvcr8nFC5TKSRD3rUCdQUvWfVmstrpknhpfdDtkELXrwGtq0l0ZnigwAw9K0ADv%2FOz49edbeQxrAnjf8HZkSjavVSUycPc7leX%2FaRfZUN1ofW&acctmode=0&pass_ticket=K5XFCnpqUHgyghRamSLrpJZesihkkOehdPru%2FXgwII8ajjZ0zhjFFOeELdXcmvMPngHysHFBaWco1M9VK5rpcQ%3D%3D&wx_header=1&fontgear=2)
