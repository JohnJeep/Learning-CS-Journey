<!--
 * @Author: JohnJeep
 * @Date: 2023-10-10 15:31:51
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-03-21 11:45:27
 * @Description: Linux clock usage
 * Copyright (c) 2025 by John Jeep, All Rights Reserved. 
-->

- [1. introduction](#1-introduction)
- [2. date](#2-date)
- [3. hwclock](#3-hwclock)
- [4. ntpd](#4-ntpd)
- [5. public time servers](#5-public-time-servers)
- [6. references](#6-references)


# 1. introduction

Linux 系统提供了多种方法来查看和设置系统时间，包括 `date` 命令、硬件时钟（`hwclock`）以及网络时间协议（NTP）服务。本文将介绍这些方法的使用。


# 2. date

`date` 命令可以用来查看和手动设置日期、时间。命令格式如下：

```bash
# 查看日期
[root@zz-scf-iot-api02 log]# date
Thu Sep 26 10:32:11 CST 2024

# 修改时间
[root@node1 ~]# date -s "20240225 20:16:00"  #yyyymmdd hh:mm:ss
Tue Feb 25 20:16:00 CST 2024

```

- **MM**：月份（两位数，01-12）
- **DD**：日期（两位数，01-31）
- **hh**：小时（两位数，00-23，24小时制）
- **mm**：分钟（两位数，00-59）
- **YYYY**：年份（四位数）


# 3. hwclock

Linux 系统的硬件时间。

```bash
# 查看硬件时间
root@DESKTOP-0S33AUT:/home/hacker# hwclock
2024-09-26 10:36:37.978831+08:00

# 以系统时间为基准，修改硬件时间。将新的系统时间写入硬件时钟（BIOS时钟）。这是为了确保系统在下次启动时使用正确的时间。
hwclock -w, --systohc   #  System Clock to Hardware Clock

# 以硬件时间为基准，修改系统时间
hwclock -s, --hctosys   # he Hardware Clock to System Clock

```


# 4. ntpd

NTP（Network Time Protocol）是一种网络协议，用于同步计算机系统的时间。`ntpd` 是 NTP 的守护进程，可以让系统的时间与远程 NTP 服务器持续保持同步。它适合长期在线的服务器，能逐步调整系统时钟，保持时间的精准性。

NTP 的使用方法请见：[NTP](./ntp.md)


# 5. public time servers

公共时间服务器是由第三方提供的 NTP 服务器，任何人都可以使用它们来同步系统时间。阿里云提供了公共时间服务器，授时信号来自 GPS、北斗两套卫星信号，并配备原子钟守时，以下7个域名提供服务，大家可以直接使用。

```
http://time1.aliyun.com
http://time2.aliyun.com
http://time3.aliyun.com
http://time4.aliyun.com
http://time5.aliyun.com
http://time6.aliyun.com
http://time7.aliyun.com

或者直接访问这个地址  time.pool.aliyun.com

// 实例
ntpdate -u time.pool.aliyun.com
```


# 6. references

- [ntp时间服务器 时间同步](https://www.cnblogs.com/centos2017/p/7896704.html)
- [Linux系统时间同步方法小结](https://www.cnblogs.com/williamjie/p/10768657.html)
