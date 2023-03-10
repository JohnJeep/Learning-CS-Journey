# crontab 用法

## 简介

crontab 命令主要用于设置命令行或者脚本周期性的执行。该命令从标准输入设备读取指令，并将其存放于文件中，以供之后读取和执行。本文主要讲述crontb命令的基本语法和配置方法。

## 命令格式

```shell
crontab [-u user] -e -l -r
```

- -u 用户名，不加 -u 参数默认为当前用户。
- -e 编辑 crontab 文件。
- -l 列出 crontab 文件中的内容。取值来源为 `/var/spool/cron`下对应的文件
- -r 删除 crontab 文件。

用法

```shell
# Example of job definition:
 .---------------- minute (0 - 59)
 |  .------------- hour (0 - 23)
 |  |  .---------- day of month (1 - 31)
 |  |  |  .------- month (1 - 12) OR jan,feb,mar,apr ...
 |  |  |  |  .---- day of week (0 - 6) (Sunday=0 or 7) OR sun,mon,tue,wed,thu,fri,sat
 |  |  |  |  |
 *  *  *  *  * command/script
```

一个定时任务的配置共包括6个字段，分别是分、时、日、月、周、命令行或脚本，每一列取值的范围或者含义如上述格式中注释描述。特别注意一点是命令行或者脚本一定要配置成绝对路径。

## 特殊字段说明

- 星号（`*`）：代表所有可能的值，例如在第1列表示每分钟执行，第2列表示每小时执行，第3列表示每天执行。
- 逗号（`,`）：可以用逗号隔开的值表示指定一个列表范围，例如，在第 1 列设置 15, 30, 45 表示在第15分钟、30分钟、45分钟执行。
- 中杠（`-`）：可以用整数之间的中杠表示一个整数范围，例如 "2-6" 表示 2, 3, 4, 5, 6
- 正斜线（`/`）：可以用正斜线指定时间的间隔频率，例如第2列设置成 "*/2"，表示每两小时执行一次。

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

- crontab每10秒执行一次`rumenz.sh`

  ```
  * * * * * /root/rumenz.sh
  * * * * * sleep 10;/root/rumenz.sh
  * * * * * sleep 20;/root/rumenz.sh
  * * * * * sleep 30;/root/rumenz.sh
  * * * * * sleep 40;/root/rumenz.sh
  * * * * * sleep 50;/root/rumenz.sh
  ```

- 每 90 分钟运行一次`rumenz.sh`

  ```
  0 0-21/3 * * * /root/rumenz.sh
  30 0-22/3 * * * /root/rumenz.sh
  ```

- 每90秒执行一次`rumenz.sh`

  ```
  */3 * * * * /root/rumenz.sh
  */3 * * * * sleep 90;/root/rumenz.sh
  ```

# Reference

- [Linux之crontab使用技巧](https://mp.weixin.qq.com/s?__biz=MzI4MDEwNzAzNg==&mid=2649459914&idx=2&sn=8f7e0735aceea33cf8117d312e1850d9&key=e459974591e3bac8f20384884c0fe290a53a9ac6358e9fc28da14268f503ad46863b05b9a889c0c144f38badd3be015a4f5decec10c4cb18ff4451985f916d5e88fda50d2ba31ac64cd477a6f254f42b136155835c7239e46eaf2bf956de1645793ed89f50ddd70f773eddb64cf2fc34ba664d2b066b9d3f9c0946570f7f74ce&ascene=0&uin=MTE2MDU5MjIzNA%3D%3D&devicetype=Windows+10+x64&version=6309001c&lang=zh_CN&countrycode=CN&exportkey=n_ChQIAhIQ5CCO7T21W4m%2FHwxeQZ3VihLgAQIE97dBBAEAAAAAAD2BFie%2FpmYAAAAOpnltbLcz9gKNyK89dVj0jSgivwYV2B3yIwV6skhUZ%2F21hP7%2FPBzb%2FRAB4wNgG%2BQrwySrMxiEetlGog7JYCky9UKJ55h9c%2B0b94pVa2DwKSj6ft6o842FlbRPuZyIUIG%2FS%2F%2BHE7urRVA3%2FWPK1tfwgpxbHYv2wLhgvcr8nFC5TKSRD3rUCdQUvWfVmstrpknhpfdDtkELXrwGtq0l0ZnigwAw9K0ADv%2FOz49edbeQxrAnjf8HZkSjavVSUycPc7leX%2FaRfZUN1ofW&acctmode=0&pass_ticket=K5XFCnpqUHgyghRamSLrpJZesihkkOehdPru%2FXgwII8ajjZ0zhjFFOeELdXcmvMPngHysHFBaWco1M9VK5rpcQ%3D%3D&wx_header=1&fontgear=2)