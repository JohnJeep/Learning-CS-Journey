<!--
 * @Author: JohnJeep
 * @Date: 2020-04-04 21:22:08
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-04-21 17:12:38
 * @Description: Scoket 套接字用法
 * Copyright (c) 2025 by John Jeep, All Rights Reserved. 
-->

# 1. socket 套接字

- NAT映射
  - 作用对象
    - 公网---私网
    - 私网---公网

- 打洞机制
  - 作用对象
    - 私网---私网

- 大端法(Big-Endians)：高地址存低字节数据，低地址存储高字节数据
- 小端法(Small-Endins)：高地址存高字节数据，低地址存储低字节数据


- `inet_pton`和`inet_ntop`函数
  - `inet_pton()` 将点分十进制字符串类型的IP地址转化为网络二进制字节序
  - `inet_ntop()` 将网络二进制字节序转化为点分十进制字符串类型的IP地址
  > 注意缩写：`p` presentation：表达式； `n` numeric：数值


- `htons、ntohs、htonl和ntohl`函数
  - 主机字节序(本地)与网络字节序之间相互转换的几组API函数，本地套接字一般按照 `小端法` 存储，网络字节序一般按照 `大端法` 存储。 
  - 注意缩写：`h`：host，`n`：net，`l`：long，`s`：short
    ```c
    #include <netinet/in.h>
    uint16_t htons(uint16_t host16 bitvalue);
    uint32_t htonl(uint32_t host32 bitvalue);
    uint16_t ntohs(uint16_t net16 bitvalue);
    uint32_t ntohl(uint32_t net32 bitvalue);
    ```

## 1.1. Socket addr struct
 ```c
 struct in_addr {
     in_addr_t  s_addr;           // 32- bit IPv4 address
                                  // network byte ordered
 }
 struct sockaddr_in {
     sa_family_t  sin_family;     // AF_INET
     in_port_t    sin_port;       // 16- bit TCP or UDP port nummber, network byte ordered
     struct in_addr    sin_addr;  // 32- bit IPv4 address, network byte ordered
     char     sin_zero[8];        // unused
 }

 struct sockaddr {
   sa_family_t  sa_family;        // address family: AF_XXX value
   char        sa_data[14];       // protocol-specific address
 }
 ```

- `struct sockaddr_in` 
  - 是网络套接字地址结构，大小为 `16字节`，定义在<netinet/in>头文件中，可用 `man 7 ip` 命令查看位置。
  - 一般我们在程序中是使用该结构体时，作为参数传递给套接字函数时需要强转为 `sockaddr` 类型，注意该结构体中 `port`和 `addr` 成员是网络序的(大端结构)。即定义时需要定义为 `struct sockaddr_in `;在调用时，需要强制转化为 `(struct sockaddr *)` 结构体类型
- `struct sockaddr`
  - 是套接字地址结构，当作为参数传递给套接字函数时，套接字地址结构总是以指针方式来使用，比如bind/accept/connect函数等。


套接字(socket)
- 网络中成对出现
- 一个文件描述符指向两个内核缓冲区(一个读、一个写)
- 包含一个IP地址和一个端口号，指定IP和端口号


## 1.2. Socket GAPI

- `socket()` 
  - 作用：创建一个套接字
  - 参数
    - `domain`: 对于IPv4，`domain` 设置为 `AF_IENT`
    - `type`: 对于TCP，面向数据流的传输协议，应设置为 `SOCK_STREAM`；对于UDP，面向报文传输的协议，应设置为 `SOCK_DGRAM`
    - `protocol`: 根据情况而定，一般设置为 `0`
  - 返回值
    - 成功: 返回指向新创建socket的文件描述符
    - 失败：返回 `-1`

- `bind()` 服务器调用 `bind` 绑定固定的网络地址和端口号 
- `listen()` 服务器允许客户端同一时间可以建立多少连接，即最多允许有多少个客户端处于连接等待状态。
- `accept()` 服务器端的套接字上接受一个连接请求。
  - 返回值
    - 成功：返回一个全新的socket文件描述符，用于和客户端通信。
    - 失败：返回 `-1`
- `connect()` 客户端调用 `connect` 与服务器建立连接


- `read()`
  - 返回值大于 `0` 时，返回实际读到的字节数。
  - 返回值等你 `0` 时，`read()` 函数数据才读完。
  - 返回值等于 `-1` 时，出现异常
    - `errno == EINTR` 时，read函数被信号中断，则需要重启或退出(quit)。
    - `errno == EAGAIN` 时，以非阻塞(EWOULDBLOCK)的方式去读，但是读到的没有数据。 
    - 出现其它的值时，则执行 `perror()` 函数显示错误提示。


- `readline()` 读取一行的内容，遇到 `\n` 则结束

> 注意点：在使用套接字函数时，需要对函数做错误检查，保证代码的鲁棒性，可以把错误检查的代码封装在一起。

- `shutdown(int sockfd, int how)` 在应用程序中执行一个半关闭的状态 
  - 参数 how 
    - `SHUT_RD` 关闭sockfd套接字上的读共能 
    - `SHUT_WR` 关闭sockfd套接字上的写共能 
    - `SHUT_RDWR` 关闭sockfd套接字上的读写共能，相当于两次调用 `shutdown()`, 第一次以 `SHUT_RD`调用，第二次以 `SHUT_WR` 调用。
  - 注意点
    - `shutdown`函数不考虑文件描述符的引用计数，直接关闭文件描述符。而 `close()`函数每次关闭一次，文件描述符的个数就减一，直到计数为 0 时，所有的进程都调用了 `close()`，套接字才全被释放。
    - 在多进程的通信中，若一个进程调用了 `shutdown(sfd, SHUT_RDWR)`，则其它的进程不能进行通信；若一个进程调用了 `close(sfd)`，将不会影响其它进程的通信。


# 2. 短连接

## 2.1. 短连接的操作步骤

建立连接——数据传输——关闭连接...建立连接——数据传输——关闭连接

1. client 向 server 发起连接请求
2. server 接到请求，双方建立连接
3. client 向 server 发送消息
4. server 回应 client
5. 一次读写完成，此时双方任何一个都可以发起 close 操作


## 2.2. 优点
短连接对于服务器来说管理较为简单，存在的连接都是有用的连接，不需要额外的控制手段。


## 2.3. 缺点
但如果客户请求频繁，将在TCP的建立和关闭操作上浪费时间和带宽。

# 3. 长连接

## 3.1. 长连接的操作步骤是：

建立连接——数据传输...（保持连接）...数据传输——关闭连接

1. client 向 server 发起连接
2. server 接到请求，双方建立连接
3. client 向 server 发送消息
4. server 回应 client
5. 一次读写完成，连接不关闭
6. 后续读写操作...
7. 长时间操作之后client发起关闭请求



## 3.2. 优点

长连接可以省去较多的TCP建立和关闭的操作，减少浪费，节约时间。

对于频繁请求资源的客户来说，较适用长连接。



## 3.3. 缺点

client与server之间的连接如果一直不关闭的话，会存在一个问题，随着客户端连接越来越多，server早晚有扛不住的时候，这时候server端需要采取一些策略，如关闭一些长时间没有读写事件发生的连接，这样可以避免一些恶意连接导致server端服务受损；如果条件再允许就可以以客户端机器为颗粒度，限制每个客户端的最大长连接数，这样可以完全避免某个蛋疼的客户端连累后端服务。



## 3.4. 什么时候用长连接，短连接

长连接多用于操作频繁，点对点的通讯，而且连接数不能太多情况，。每个TCP连接都需要三步握手，这需要时间，如果每个操作都是先连接，再操作的话那么处理速度会降低很多，所以每个操作完后都不断开，下次处理时直接发送数据包就OK了，不用建立TCP连接。例如：数据库的连接用长连接，如果用短连接频繁的通信会造成socket错误，而且频繁的socket 创建也是对资源的浪费。


而像WEB网站的http服务一般都用短链接，因为长连接对于服务端来说会耗费一定的资源，而像WEB网站这么频繁的成千上万甚至上亿客户端的连接用短连接会更省一些资源，如果用长连接，而且同时有成千上万的用户，如果每个用户都占用一个连接的话，那可想而知吧。所以并发量大，但每个用户无需频繁操作情况下需用短连好


# 4. 心跳

跳机制的原理很简单：客户端每隔N秒向服务端发送一个心跳消息，服务端收到心跳消息后，回复同样的心跳消息给客户端。如果服务端或客户端在M秒（M>N）内都没有收到包括心跳消息在内的任何消息，即心跳超时，我们就认为目标TCP连接已经断开了。


# 5. 轮询

短轮询：浏览器发起一个“询问”请求，服务器无论有无新数据，都立即响应（有就返回新数据，没有就返回一个表示’空’的自定义数据格式），一个HTTP连接结束。 


长轮询：长轮询的经典实现 —— Comet：基于 HTTP 长连接的“服务器推”技术。
- 浏览器发起一个“询问”请求，当没有新数据时，服务器端并不立即响应，而是等待数据，当有新数据产生时，才向浏览器响应，一个HTTP连接结束。
- 补充： 当服务端没有数据更新的时候，连接会保持一段时间周期知道数据或者状态改变或者过期，依次减少无效的客户端和服务端的交互。
- 补充： 当服务端数据变更频繁的话，这种机制和定时轮询毫无区别。

# 6. References

- 深入理解基本套接字编程: https://www.cnblogs.com/luoxn28/p/5819798.html
- 长连接和短连接详细解析，值得阅读: https://cloud.tencent.com/developer/article/1470024
- tcp长连接和短连接: https://www.cnblogs.com/georgexu/p/10909814.html
- 长连接与短连接 同步与异步:https://www.cnblogs.com/biGpython/archive/2011/11/17/2252401.html
- websocket长链接和短连接: https://www.cnblogs.com/miaozhihang/p/9518584.html
