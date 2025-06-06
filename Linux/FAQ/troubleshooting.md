<!--
 * @Author: JohnJeep
 * @Date: 2021-05-19 12:17:57
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-04-05 11:32:00
 * @Description: linux 中常见问题排查
 * Copyright (c) 2025 by John Jeep, All Rights Reserved. 
-->

# 1. linux 日志

## 1.1. 分类

- 内核日志

  由系统服务 `syslog` 或 `rsyslog` 同一管理，根据配置文件 `/etc/syslog.conf` 或  `/etc/rsyslog.conf` 中的内容，决定将内核消息和各种系统程序的信息记录什么位置。

- 用户日志

  用于记录 Linux 系统用户登录、登出系统的相关信息，包括用户名、登录的终端、登录时间、正在使用的进程操作等。

- 应用程序日志

  第三方应用程序的日志信息。比如公司开发的程序有一套日志记录的模块，来记录程序运行过程中的各种事件信息，便于出错时问题的排查。

## 1.2. 日志文件解读

Linux 系统本身和大部分服务器程序的日志文件默认情况下都放置在目录 `/var/log` 中。

- `/var/log/messages`：公共日志文件，记录 Linux 内核消息及各种应用程序的消息和事件信息。这个文件包含了系统的重要事件、错误（IO 错误、网络错误）、程序故障、警告和其他相关信息等，对于系统管理员来说是一个非常有用的日志文件。

  `/var/log/messages` 是一个传统的日志文件路径，通常由 Syslog 守护进程（syslogd 或 rsyslogd）负责管理。Syslog 是一个标准的系统日志记录工具，它可以接收来自系统和应用程序的日志消息，并将它们写入不同的日志文件。`/var/log/messages` 是其中一个主要的日志文件，记录了来自许多不同来源的日志消息。

  在许多 Linux 发行版中，`/var/log/messages` 文件已被替换或合并到其他日志文件中，例如 `/var/log/syslog`（Debian/Ubuntu 等）、`/var/log/messages`（CentOS/RHEL 等）等。因此，实际的日志路径可能因发行版和系统配置而有所不同。

  如果你想查看系统日志和消息，可以使用 `cat`、`less`、`tail` 等命令来查看 `/var/log/messages` 文件或对应的日志文件，具体的命令会因发行版而有所不同。例如：

  ```sh
  cat /var/log/messages  # 查看 messages 文件的全部内容
  less /var/log/messages # 使用 less 分页查看 messages 文件
  tail -n 50 /var/log/messages # 查看 messages 文件的最后 50 行日志
  ```

  请注意，查看日志通常需要管理员权限，你可能需要使用 `sudo` 命令来查看日志文件的内容。

- `/var/log/cron`：记录 crond 计划任务产生的信息。

- `/var/log/dmesg`：在系统启动时，会在屏幕上显示许多与硬件有关的信息，此文件就是记录系统上次启动时产生的这些信息，包含内核缓冲信息（kernel ring buffer）。用 `dmesg` 命令可查看本次系统启动时与使件有关的信息，以及内核缓冲信息。

- `/var/log/mailog`：记录进入或发出系统的电子邮件的信息。

- `/var/log/boot.log`：记录系统启动时的软件日志信息。

- `/var/log/secure`：记录用户远程登录、认证过程中的信息。

- `/var/log/wtmp`：记录系统所有登录和登出纪录，可用 `last` 命令查看。

- `/var/log/btmp` ：记录错误登录系统的日志信息，可用 `lastb` 命令查看。

- `/var/log/lastlog` ：记录最近成功登录的事件和最后一次不成功的登录事件，可用 `lastlog` 命令查看。



# 2. 网络故障排查

## 2.1. 检查网线

`ethtool ethx` 命令查看某一网卡的链路是否物理连通，ethx 为网卡的名称。

不知道当前系统的网卡是什么。用 `ifconfig` 命令查看。

```
[root@CentOS7 ~]# ifconfig
ens33: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
        inet 192.168.0.180  netmask 255.255.255.0  broadcast 192.168.0.255
        inet6 fe80::675b:99f5:ced9:26fa  prefixlen 64  scopeid 0x20<link>
        ether 00:0c:29:cc:b7:5e  txqueuelen 1000  (Ethernet)
        RX packets 12028  bytes 2033103 (1.9 MiB)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 1832  bytes 473741 (462.6 KiB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

lo: flags=73<UP,LOOPBACK,RUNNING>  mtu 65536
        inet 127.0.0.1  netmask 255.0.0.0
        inet6 ::1  prefixlen 128  scopeid 0x10<host>
        loop  txqueuelen 1000  (Local Loopback)
        RX packets 32  bytes 2592 (2.5 KiB)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 32  bytes 2592 (2.5 KiB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0
```

可以看到 第一个就是网卡的名称，叫 `ens33`。

```
// 查看 网卡 ens33 的状态
[root@CentOS7 ~]# ethtool ens33
Settings for ens33:
        Supported ports: [ TP ]
        Supported link modes:   10baseT/Half 10baseT/Full
                                100baseT/Half 100baseT/Full
                                1000baseT/Full
        Supported pause frame use: No
        Supports auto-negotiation: Yes
        Supported FEC modes: Not reported
        Advertised link modes:  10baseT/Half 10baseT/Full
                                100baseT/Half 100baseT/Full
                                1000baseT/Full
        Advertised pause frame use: No
        Advertised auto-negotiation: Yes
        Advertised FEC modes: Not reported
        Speed: 1000Mb/s
        Duplex: Full
        Port: Twisted Pair
        PHYAD: 0
        Transceiver: internal
        Auto-negotiation: on
        MDI-X: off (auto)
        Supports Wake-on: d
        Wake-on: d
        Current message level: 0x00000007 (7)
                               drv probe link
        Link detected: yes

```

 

## 2.2. 检查网卡状态

查看网卡驱动状态

```
[root@CentOS7 ~]# ethtool -i ens33
driver: e1000
version: 7.3.21-k8-NAPI
firmware-version:
expansion-rom-version:
bus-info: 0000:02:01.0
supports-statistics: yes
supports-test: yes
supports-eeprom-access: yes
supports-register-dump: yes
supports-priv-flags: no
```

`lsmod` 查看网络各个模块的加载。

`lspci` 显示系统中所有 PCI 总线设备或连接到该总线上的所有设备的工具。具体使用参考：[lspci命令详解](https://www.cnblogs.com/machangwei-8/p/10403495.html)

## 2.3. 查看网卡配置文件

配置文件路径：` /etc/sysconfig/network-scripts/ifcfg-ens33`




