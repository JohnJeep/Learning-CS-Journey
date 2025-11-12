<!--
 * @Author: JohnJeep
 * @Date: 2021-02-08 21:59:39
 * @LastEditTime: 2025-04-04 19:24:51
 * @LastEditors: JohnJeep
 * @Description: 探究内存管理
-->

# History(历史)

Doug Lea自1986年开始研究malloc算法，他的作品被称为 DL Malloc，目前linux中的glibc的malloc算法就是直接来自Doug Lea，其它平台的malloc的实现或多或少受到DL的影响。



# 内存管理层级

1. OS（操作系统），Windows 系统的的 `HeapAlloc` 和 `VirtualAlloc`；
2. GUNC++/CRT 编译器的 `malloc()`
3. C++ 标准库的 `Allocator`
4. C++ 应用程序层面的内存 API 函数。






# 内存库作品

- tcmalloc：稳定，占用内存更低。
- jemalloc性能更高，占用内存更高

# Bibliography(书目)
- STL源码剖析(侯捷)
- Small Memory Software(James Noble & Charles Weir)
- Modern C++ Design General Programming and Design Patterns Applied(Andrei Alexandrescu)



# Reference(参考)
- [Doug Lea's Home Page](http://gee.cs.oswego.edu/): Malloc算法的发明者网站。
- [Wikipedia Malloc](https://en.wikipedia.org/wiki/C_dynamic_memory_allocation): 很权威的Wikipedia解释Malloc的用法。