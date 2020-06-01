<!--
 * @Author: JohnJeep
 * @Date: 2020-05-12 21:34:44
 * @LastEditTime: 2020-06-01 13:52:00
 * @LastEditors: Please set LastEditors
 * @Description: 物理内存之外: 策略部分(Policies)
--> 

### 问题(crux of the problem)
- OS如何决定从内存中踢出哪个页？


### Cache Management
- AMAT(average memory access time): 内存平均访问时间
  - $T_M$：the cost of accessing memory
  - $T_D$：the cost of accessing disk
  - $P_Miss$：the probability of not finding the data in the cache (a miss)
$$AMAT = T_M + (P_Miss · T_D)$$


### The Optimal Replacement Policy(最优替换策略)
- 最优替换策略的结果：使总体未命中TLB的数量最少。
- 最优替换策略只能作为理想的策略作为比较，在实际中很难实现。
- 3种缓存未命中的类型
  - compulsory miss：强制性未命中
  - capacity miss：容量未命中
  - conflict miss：冲突未命中

- 采用FIFO和Random策略都可能会踢出重要的page，这个page也许会马上被引用。


### 基于历史信息的算法去实现一些策略 
- **LRU**(Least-Recently-Used)：最少最近使用
- LFU(Least-Frequently-Used): 最不经常使用
- MFU(Most-Frequently-Used): 最经常使用
- MRU(Most-Recently-Used): 最近使用
- 时间局部性(temporal locality)：近期访问的页可能会在不久的将来再次被访问。
- 空间局部性(spatial locality): 如果page P被访问，可能围绕它的page也会被访问。
- 使用位(use bit)，也叫引用位(reference bit)
- 修改位(modified bit)，也叫脏位(dirty bit)


- 如何实现LRU替换策略？
  - OS利用use bit来实现Approximating LRU。OS中的所有page都放在一个circular list中，开始时，时钟指针(clock hand)指向某个特定的page，当进行page replacement时，OS检查当前指向的page的use bit是0还是1；若是1，则OS指向的page最近被使用，不适合被替换，然后将当前page的use bit设置为0，时钟指针递增到下一个page，一直持续找到一个use bit为0的page；若是0，则OS指向的page最近没有被使用，可以进行替换。


- 采用时钟算法(clock algorithm)不是实现近似LRU的唯一方法。只要任何周期性的清除use bit，然后通过区分use bit 是0还是1来判断该替换哪个page。
- 加入dirty bit后的时钟算法。扫描没有被使用又没有被修改的page首先踢出(evict)，若没有找到这种page，在查找被修改过且没有被使用的page。
