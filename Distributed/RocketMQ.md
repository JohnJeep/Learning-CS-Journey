Broker 每隔 30s 向 Name Server 发送心跳，Name Server 如果 120s 没有收到心跳，就会判断 Broker 宕机了。





ConsumeQueue 中的元素内容

- 前 8 个 bytes 记录消息在 CommitLog 中的偏移量。
- 中间 4 个 bytes 记录消息消息大小。
- 最后 8 个 bytes 记录消息中 tag 的 hashcode。

