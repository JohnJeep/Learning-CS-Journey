<!--
 * @Author: JohnJeep
 * @Date: 2023-07-10 09:54:50
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-04-04 19:18:33
 * @Description: Go 底层原理剖析
 * Copyright (c) 2023 by John Jeep, All Rights Reserved. 
-->

## 程序是怎样跑起来的

曹大的 《Go 程序的启动流程》和@全成的 《Go 程序是怎样跑起来的》



# HTTP



# Map

1.24 版本 Go map 的底层实现使用 `Swisstable` 实现。

Swiss Table是由Google工程师于2017年开发的一种高效哈希表实现，旨在优化内存使用和提升性能，解决Google内部代码库中广泛使用的 `std::unordered_map` 所面临的性能问题。[Google工程师Matt Kulukundis在2017年CppCon大会上详细介绍了他们在Swiss Table上的工作](https://www.youtube.com/watch?v=ncHmEUmJZf4)：



## effective

`Swiss Table` 在查询、插入和删除操作上均提升了20%至50%的性能，尤其是在处理大 `hashmap` 时表现尤为突出；迭代性能提升了10%；内存使用减少了0%至25%，并且不再消耗额外内存。




# References

- Github golang-internals-resources: https://github.com/emluque/golang-internals-resources
- Github learning go: https://github.com/yangwenmai/learning-golang
- Go 源码阅读工具： https://mp.weixin.qq.com/s/E2TL_kcbVcRJ0CnxwbXWLw
- Github golang-notes, 源码剖析：https://github.com/cch123/golang-notes/tree/master
- Go Context 并发编程简明教程：https://geektutu.com/post/quick-go-context.html

---

HTTP

Golang 使用系列---- Net/Http 应用层: https://kingjcy.github.io/post/golang/go-net-http

---

map

- Go map使用Swiss Table重新实现，性能最高提升近50%: https://tonybai.com/2024/11/14/go-map-use-swiss-table/
- Swiss Tables and `absl::Hash`: https://abseil.io/blog/20180927-swisstables

- 简单了解下最近正火的 SwissTable: https://www.cnblogs.com/apocelipes/p/17562468.html

