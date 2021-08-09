<!--
 * @Author: your name
 * @Date: 2021-08-09 08:23:06
 * @LastEditTime: 2021-08-09 16:16:25
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \Learning-Computer-Science-Journey\temp.md
-->
短连接的操作步骤是：

建立连接——数据传输——关闭连接...建立连接——数据传输——关闭连接

- client 向 server 发起连接请求
- server 接到请求，双方建立连接
- client 向 server 发送消息
- server 回应 client
- 一次读写完成，此时双方任何一个都可以发起 close 操作




长连接的操作步骤是：

建立连接——数据传输...（保持连接）...数据传输——关闭连接


- client 向 server 发起连接
- server 接到请求，双方建立连接
- client 向 server 发送消息
- server 回应 client
- 一次读写完成，连接不关闭
- 后续读写操作...
- 长时间操作之后client发起关闭请求


# 参考

https://www.cnblogs.com/georgexu/p/10909814.html
https://www.cnblogs.com/biGpython/archive/2011/11/17/2252401.html
https://cloud.tencent.com/developer/article/1470024
https://www.cnblogs.com/miaozhihang/p/9518584.html




