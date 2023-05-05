## gRPC调优

gRPC 默认的参数对于传输大数据块来说不够友好，我们需要进行特定参数的调优。

1. `MaxSendMsgSize`: gRPC最大允许发送的字节数，默认 4MiB，如果超过了 gRPC 会报错。Client 和 Server 我们都调到 4GiB。

2. `MaxRecvMsgSize`g: RPC最大允许接收的字节数，默认 4MiB，如果超过了 gRPC 会报错。Client  和 Server 我们都调到 4GiB。

3. `InitialWindowSize`: 基于 Stream 的滑动窗口，类似于 TCP  的滑动窗口，用来做流控，默认  64KiB，吞吐量上不去，Client 和 Server 我们调到 1GiB。

4. `InitialConnWindowSize`: 基于 Connection 的滑动窗口，默认`16 * 64KiB`，吞吐量上不去，Client 和Server 我们也都调到 1GiB。

5. `KeepAliveTime`: 每隔 KeepAliveTime 时间，发送 PING 帧测量最小往返时间，确定空闲连接是否仍然有效，我们设置为10S。

6. `KeepAliveTimeout`: 超过 KeepAliveTimeout，关闭连接，我们设置为 3S。

7. `PermitWithoutStream`: 如果为 true，当连接空闲时仍然发送 PING 帧监测，如果为 false，则不发送忽略。我们设置为 true。

8. `MaxConcurrentStreams`: 一条gRPC连接允许并发的发送和接收多个Stream。数值小了，吞吐量上不去。Golang的服务端默认是100。

   > **注意**：虽然一条连接上面能够处理更多的请求了，但一条连接远远是不够的。一条连接通常只有一个线程来处理，所以并不能充分利用服务器多核的优势。同时，每个请求编解码还是有开销的，所以用一条连接还是会出现瓶颈。