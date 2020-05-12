### 内存操作API

1. 动态分配内存常见误区
   - 忘记分配内存（Forgetting To Allocate Memory）
   ```
    char *src = "hello";
    char *dst; // oops! unallocated
    strcpy(dst, src); // segfault and die
   ```
   - 没有足够的内存，也称为缓冲溢出
   ```
    为字符串声明空间：采用 malloc(strlen(src) + 1) 用法，一边为字符串结束符留出空间。
    char *src = "hello";
    char *dst = (char *) malloc(strlen(src) + 1);
    strcpy(dst, src); // work properly
   ```
   - 忘记初始化分配的内存。忘记在新申请的数据类型中填充一些值，导致程序最终会遇到未初始化的读取，从堆中读取一些未知的数据。
        
   - 忘记释放内存，即内存泄露（memory leak）。如果仍然拥有对某块内存的引用，那么垃圾收集器就不会释放它。
   - 在用完之前释放内存，这种错误称为悬挂指针（dangling pointer）。可能会导致程序崩溃或者覆盖有效的内存。
   - 反复释放内存（Freeing Memory Repeatedly），也被称为重复释放（double free）。导致结果未定义。
   - 错误的调用free()。



2. 查看内存泄露工具
   - Purify：现在是商业产品
   - valgrind：开源工具。（参考：[应用 Valgrind 发现 Linux 程序的内存问题](https://www.ibm.com/developerworks/cn/linux/l-cn-valgrind/index.html)）


3. 底层操作系统支持
   - malloc()与free()函数不是系统调用，而是库调用。malloc库管理虚拟地址空间内的空间。
   - malloc()与free()基于**brk**或**sbrk**系统调用之上。**brk**作用：改变程序分段的位置，堆结束的位置。
   - 调用mmap()，从操作系统获取内存。mmap()在程序中创建一个匿名内存区域，这个区域不与任何特定文件相关联，而是与交换空间（swap space）相关联。

