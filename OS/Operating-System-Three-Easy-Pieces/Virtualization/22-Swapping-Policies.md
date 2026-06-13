<!--
 * @Author: JohnJeep
 * @Date: 2020-05-12 21:34:44
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-05-31 19:58:36
 * @Description: Swapping-Policies
 * Copyright (c) 2026 by John Jeep, All Rights Reserved. 
-->

## 问题(crux of the problem)

- OS 如何决定从内存中踢出哪个页？


## Cache Management

- AMAT(average memory access time): 内存平均访问时间
  - $T_M$：the cost of accessing memory
  - $T_D$：the cost of accessing disk
  - $P_Miss$：the probability of not finding the data in the cache (a miss)
$$AMAT = T_M + (P_Miss · T_D)$$


## The Optimal Replacement Policy(最优替换策略)

- 最优替换策略的结果：使总体未命中 TLB 的数量最少。
- 最优替换策略只能作为理想的策略作为比较，在实际中很难实现。
- 3 种缓存未命中的类型
  - compulsory miss：强制性未命中
  - capacity miss：容量未命中
  - conflict miss：冲突未命中
- 采用 FIFO 和 Random 策略都可能会踢出重要的 page，这个 page 也许会马上被引用。


## 基于历史信息的算法去实现一些策略 

- **LRU**(Least-Recently-Used)：最少最近使用
- LFU(Least-Frequently-Used): 最不经常使用
- MFU(Most-Frequently-Used): 最经常使用
- MRU(Most-Recently-Used): 最近使用
- 时间局部性(temporal locality)：近期访问的页可能会在不久的将来再次被访问。
- 空间局部性(spatial locality): 如果 page P 被访问，可能围绕它的 page 也会被访问。
- 使用位(use bit)，也叫引用位(reference bit)
- 修改位(modified bit)，也叫脏位(dirty bit)


- 如何实现 LRU 替换策略？
  - OS 利用 use bit 来实现 Approximating LRU。OS 中的所有 page 都放在一个 circular list 中，开始时，时钟指针(clock
    hand)指向某个特定的 page，当进行 page
    replacement 时，OS 检查当前指向的 page 的 use bit 是 0 还是 1；若是 1，则 OS 指向的 page
    最近被使用，不适合被替换，然后将当前 page 的 use bit 设置为
    0，时钟指针递增到下一个 page，一直持续找到一个 use bit 为 0 的 page；若是 0，则 OS 指向的 page
    最近没有被使用，可以进行替换。


- 采用时钟算法(clock algorithm)不是实现近似 LRU 的唯一方法。只要任何周期性的清除 use bit，然后通过区分 use bit 是 0
  还是 1 来判断该替换哪个 page。
- 加入 dirty bit 后的时钟算法。扫描没有被使用又没有被修改的 page 首先踢出(evict)，若没有找到这种
  page，在查找被修改过且没有被使用的 page。
