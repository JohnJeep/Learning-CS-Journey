# linux 系统如何查看CPU信息、机器型号、内存信息等

## 系统

```shell
# uname -a        # 查看内核/操作系统/CPU信息
# head -n 1 /etc/issue  # 查看操作系统版本
# cat /proc/cpuinfo   # 查看CPU信息
# hostname        # 查看计算机名
# lspci -tv       # 列出所有PCI设备
# lsusb -tv       # 列出所有USB设备
# lsmod         # 列出加载的内核模块
# env          # 查看环境变量
```

## 资源

```shell
# free -m        # 查看内存使用量和交换区使用量
# df -h         # 查看各分区使用情况
# du -sh <目录名>    # 查看指定目录的大小
# grep MemTotal /proc/meminfo  # 查看内存总量
# grep MemFree /proc/meminfo  # 查看空闲内存量
# uptime         # 查看系统运行时间、用户数、负载
# cat /proc/loadavg   # 查看系统负载
```

## 磁盘和分区

```shell
# mount | column -t   # 查看挂接的分区状态
# fdisk -l        # 查看所有分区
# swapon -s       # 查看所有交换分区
# hdparm -i /dev/hda   # 查看磁盘参数(仅适用于IDE设备)
# dmesg | grep IDE    # 查看启动时IDE设备检测状况
```

## 网络

```shell
# ifconfig        # 查看所有网络接口的属性
# iptables -L      # 查看防火墙设置
# route -n        # 查看路由表
# netstat -lntp     # 查看所有监听端口
# netstat -antp     # 查看所有已经建立的连接
# netstat -s       # 查看网络统计信息
```

## 进程

```shell
# ps -ef         # 查看所有进程
# top          # 实时显示进程状态
```

## 用户

```shell
# w           # 查看活动用户
# id <用户名>      # 查看指定用户信息
# last          # 查看用户登录日志
# cut -d: -f1 /etc/passwd  # 查看系统所有用户
# cut -d: -f1 /etc/group  # 查看系统所有组
# crontab -l       # 查看当前用户的计划任务
```

## 服务

```shell
# chkconfig --list    # 列出所有系统服务
# chkconfig --list | grep on  # 列出所有启动的系统服务
```

## 程序

```shell
# rpm -qa        # 查看所有安装的软件包
```

**查看CPU信息（型号）** 

```shell
# cat /proc/cpuinfo | grep name | cut -f2 -d: | uniq -c 
   8 Intel(R) Xeon(R) CPU      E5410  @ 2.33GHz 
(看到有8个逻辑CPU, 也知道了CPU型号) 

# cat /proc/cpuinfo | grep physical | uniq -c 
   4 physical id   : 0 
   4 physical id   : 1 
(说明实际上是两颗4核的CPU) 

# getconf LONG_BIT 
  32 
(说明当前CPU运行在32bit模式下, 但不代表CPU不支持64bit) 

# cat /proc/cpuinfo | grep flags | grep ' lm ' | wc -l 
  8 
(结果大于0, 说明支持64bit计算. lm指long mode, 支持lm则是64bit) 
```

## 内存信息 

```shell
# cat /proc/meminfo 

# uname -a 
Linux euis1 2.6.9-55.ELsmp #1 SMP Fri Apr 20 17:03:35 EDT 2007 i686 i686 i386 GNU/Linux 
(查看当前操作系统内核信息) 

# cat /etc/issue | grep Linux 
Red Hat Enterprise Linux AS release 4 (Nahant Update 5) 
(查看当前操作系统发行版信息) 
```

## 机器型号

```shell
# dmidecode | grep "Product Name" 
```

## 网卡信息 

```shell
# dmesg | grep -i eth
```

