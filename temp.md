短连接的操作步骤是：
建立连接——数据传输——关闭连接...建立连接——数据传输——关闭连接

    client 向 server 发起连接请求
    server 接到请求，双方建立连接
    client 向 server 发送消息
    server 回应 client
    一次读写完成，此时双方任何一个都可以发起 close 操作




长连接的操作步骤是：
建立连接——数据传输...（保持连接）...数据传输——关闭连接


    client 向 server 发起连接
    server 接到请求，双方建立连接
    client 向 server 发送消息
    server 回应 client
    一次读写完成，连接不关闭
    后续读写操作...
    长时间操作之后client发起关闭请求




https://www.cnblogs.com/georgexu/p/10909814.html
https://www.cnblogs.com/biGpython/archive/2011/11/17/2252401.html
https://cloud.tencent.com/developer/article/1470024
https://www.cnblogs.com/miaozhihang/p/9518584.html

https://cryfeifei.top/2020/05/30/qt-ji-chu-jiao-cheng/

1、3s 内没有发送数据，平台主动断开连接
2、什么时候触发下次短连接？
     client 向 server 发送数据时，才需要建立连接  do_trasport()


