<!--
 * @Author: JohnJeep
 * @Date: 2025-03-26 11:31:13
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-03-26 15:28:31
 * @Description: How to use network tools
 * Copyright (c) 2025 by John Jeep, All Rights Reserved. 
-->

# Network Tools


## mtr 

mtr 网络侦测工具


## traceroute 


## tcpdump


## ip rout


## curl


## wget


## netstat

- `-a` 查看所有的信息
- `-at`  查看TCP包相关的信息
- `-au`  查看UDP包相关的信息
- `-tnl` 查看监听的程序
- `-ano`显示套接字
- `- r` 或 `route print` 查看路由表
- `-an` 查看当前网络的连接会话


## ARP

- -a 查看apr缓存
- arp -d 清除apr缓存


## ipconfig

-  `ipconfig /all` 显示完整配置信息 


## nc

`nc` 是 netcat 的缩写，从命令行跨网络读取和写入数据。 
- 用途
  - 侦听任意端口，以TCP/UDP 方式
  - 端口扫描
  - 传输文件
  - 测速  


## ab

ab 命令全称为：Apache bench，是 Apache 自带的压力测试工具，也可以测试 Nginx、IIS 等其他 Web 服务器:

- `-n` 总共的请求数
- `-c` 并发的请求数
- `-t` 测试所进行的最大秒数，默认值 为 50000
- `-p` 包含了需要的 POST 的数据文件
- `-T` POST 数据所使用的 Content-type 头信息

示例
```shell
# 每次发送1000并发的请求数，请求数总数为5000。
ab -n 1000 -c 5000 http://127.0.0.1/ 
```

测试前需要安装 `sudo apt install apache2-utils`