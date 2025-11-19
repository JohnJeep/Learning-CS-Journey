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

好的，我们来详细解释一下 `tcpdump` 这个强大的网络工具。

### 一、tcpdump 是什么？

**tcpdump** 是一个运行在**命令行**下的网络抓包（数据包分析）工具。它允许用户捕获和显示经过计算机上某个或多个网络接口的数据包头部或完整内容。

常用它来：

*   **诊断网络问题**：比如连接失败、速度慢等。
*   **调试网络应用**：检查客户端和服务器之间到底发送了什么数据。
*   **学习网络协议**：直观地看到TCP/IP等协议是如何工作的。
*   **进行安全分析**：检测网络中是否存在可疑或恶意的流量。

它的工作原理是，将网卡设置为“混杂模式”，在这种模式下，网卡会接收所有流经它的数据包，而不仅仅是发给它的数据包。

**重要提示**：由于 `tcpdump` 功能强大，通常需要 `root` 权限或 `sudo` 权限才能运行。在生产环境中使用时，也应遵守相关的隐私和安全政策。

---

### 二、常见用法和命令行选项

`tcpdump` 的命令行语法非常灵活，通过组合不同的选项和过滤表达式，可以实现精确的抓包。

#### 1. 基本命令格式

```bash
TCPDUMP(8)          System Manager's Manual                                       

NAME
       tcpdump - dump traffic on a network

SYNOPSIS
       tcpdump [ -AbdDefhHIJKlLnNOpqStuUvxX# ] [ -B buffer_size ]
               [ -c count ] [ --count ] [ -C file_size ]
               [ -E spi@ipaddr algo:secret,...  ]
               [ -F file ] [ -G rotate_seconds ] [ -i interface ]
               [ --immediate-mode ] [ -j tstamp_type ] [ -m module ]
               [ -M secret ] [ --number ] [ --print ] [ -Q in|out|inout ]
               [ -r file ] [ -s snaplen ] [ -T type ] [ --version ]
               [ -V file ] [ -w file ] [ -W filecount ] [ -y datalinktype ]
               [ -z postrotate-command ] [ -Z user ]
               [ --time-stamp-precision=tstamp_precision ]
               [ --micro ] [ --nano ]
               [ expression ]

tcpdump [选项] [过滤表达式]
```

#### 2. 常用选项

| 选项                | 全称              | 说明                                                         |
| :------------------ | :---------------- | :----------------------------------------------------------- |
| `-i any`            | --interface       | 监听**所有**可用的网络接口。                                 |
| `-i eth0`           | --interface       | 监听指定的网络接口，如 `eth0`, `en0`（Mac）, `wlan0`等。     |
| `-n`                |                   | **不**将地址（如IP）转换为主机名。禁用DNS解析，能显著提高抓包速度并显示IP本身。 |
| `-nn`               |                   | 在 `-n` 的基础上，**不**将端口号转换为服务名称（如不把80端口显示为`http`）。 |
| `-c 10`             | --count           | 只捕获指定数量的数据包（如10个），然后自动停止。             |
| `-A`                |                   | 以ASCII格式打印每个数据包的内容（不包括链路层头部）。适用于查看文本协议（如HTTP）。 |
| `-X`                |                   | 同时以**十六进制**和**ASCII**格式打印数据包的内容。非常适合查看协议细节和二进制数据。 |
| `-v`, `-vv`, `-vvv` | --verbose         | 显示更详细（冗余）的信息。例如，`-vvv` 会显示最全面的信息。  |
| `-s 0`              | --snapshot-length | 设置抓取数据包的**快照长度**。`-s 0` 会抓取完整的数据包（最大65535字节），确保你不会丢失信息。 |
| `-w file.pcap`      | --write           | **将原始数据包写入文件**，而不是在屏幕上显示。文件后缀通常是 `.pcap`。 |
| `-r file.pcap`      | --read            | **从文件中读取**数据包，而不是从网络接口。用于分析之前保存的抓包文件。 |
| `-q`                | --quiet           | 安静模式，打印更少的协议信息，输出更简洁。                   |

#### 3. 过滤表达式

这是 `tcpdump` 最强大的功能之一，它允许你精确地过滤出你感兴趣的数据流。表达式由 **限定词** 和 **值** 组成。

**常见限定词：**

| 类型     | 限定词                 | 例子                             | 说明                                     |
| :------- | :--------------------- | :------------------------------- | :--------------------------------------- |
| **主机** | `host`                 | `host 192.168.1.1`               | 过滤与特定主机（IP或主机名）相关的流量。 |
| **网络** | `net`                  | `net 192.168.1.0/24`             | 过滤整个网段的流量。                     |
| **端口** | `port`                 | `port 80`                        | 过滤特定端口的流量。                     |
| **协议** | `tcp`, `udp`, `icmp`等 | `tcp`                            | 过滤特定协议的流量。                     |
| **方向** | `src`, `dst`           | `src 192.168.1.1`, `dst port 53` | 过滤源（src）或目标（dst）。             |

**逻辑运算符：**

*   `and` 或 `&&`：与
*   `or` 或 `||`：或
*   `not` 或 `!`：非

---

### 三、常见用法示例

假设你的机器IP是 `192.168.1.100`。

1.  **捕获所有接口的所有流量（信息量巨大，慎用）**
    ```bash
    sudo tcpdump -i any
    ```

2.  **捕获指定网卡的流量（如 `eth0`）**
    ```bash
    sudo tcpdump -i eth0
    ```

3.  **只捕获与特定主机的流量（禁用DNS解析）**
    ```bash
    sudo tcpdump -n host 8.8.8.8
    ```

4.  **只捕获HTTP（端口80）的流量**
    ```bash
    sudo tcpdump -n port 80
    ```

5.  **捕获来自特定源IP的流量**
    ```bash
    sudo tcpdump -n src 192.168.1.50
    ```

6.  **捕获发往特定目标端口的流量（如DNS）**
    ```bash
    sudo tcpdump -n dst port 53
    ```

7.  **组合过滤：捕获主机 `1.2.3.4` 的HTTP或HTTPS流量**
    ```bash
    sudo tcpdump -n host 1.2.3.4 and \(port 80 or port 443\)
    ```
    *注意：括号需要转义。*

8.  **捕获除了PING（ICMP）之外的所有流量**
    ```bash
    sudo tcpdump -n not icmp
    ```

9.  **捕获数据包并显示内容（用于查看明文HTTP请求）**
    ```bash
    sudo tcpdump -n -A port 80
    ```

10. **将抓包结果保存到文件（便于后续用Wireshark分析）**
    ```bash
    sudo tcpdump -i any -s0 -w my_capture.pcap
    ```

11. **从文件读取并分析抓包结果**
    ```bash
    tcpdump -r my_capture.pcap -n
    ```

### 四、输出格式解读

一个典型的TCP数据包输出如下：
```
13:45:30.123456 IP 192.168.1.100.58932 > 93.184.216.34.80: Flags [S], seq 123456789, win 65535, options [mss 1460], length 0
```

*   `13:45:30.123456`：时间戳。
*   `IP`：协议类型（这里是IPv4）。
*   `192.168.1.100.58932`：源IP地址和端口号。
*   `>`：数据流向。
*   `93.184.216.34.80`：目标IP地址和端口号（这里是example.com的HTTP服务）。
*   `Flags [S]`：TCP标志位。`[S]` 表示SYN包（连接建立请求），其他常见标志有 `[P]` (PSH，推送数据), `[F]` (FIN，连接终止), `[R]` (RST，连接重置), `[.]` (ACK，确认)。
*   `seq 123456789`：TCP序列号。
*   `win 65535`：窗口大小。
*   `length 0`：数据段长度。

### 总结

`tcpdump` 是网络领域的“瑞士军刀”，虽然它是命令行工具，学习曲线稍陡，但一旦掌握，对于理解网络通信、排查问题具有不可替代的价值。建议从简单的过滤条件开始练习，逐步组合使用，并结合 `-w` 和 `-r` 选项与图形化工具Wireshark配合使用，以达到最佳的分析效果。



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