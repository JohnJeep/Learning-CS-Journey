# 思考

- 如果使用了 Redis，每次 Redis 操作的耗时是多少？对于大批量操作的场景，Redis 有使用 Pipeline 做优化吗？Redis 集群是否有考虑不兼容的操作方式？如何实现高性能的等价操作？
- Redis 集群的内存用量大小，有无监控？
- Redis 前面是否需要多加一层本地 Cache，直接在服务本地处理（如限制某个 token 一定时间只能使用一次，一个用户一天只能投一次票等等）