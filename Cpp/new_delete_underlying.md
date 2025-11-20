<!--
 * @Author: JohnJeep
 * @Date: 2025-10-22 11:13:29
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-11-20 12:47:14
 * @Description: new/delete 底层是怎样用的？
 * Copyright (c) 2025 by John Jeep, All Rights Reserved. 
-->

- [1. 整体概括](#1-整体概括)
- [2. 应用层 → C++ 运行时](#2-应用层--c-运行时)
- [3. C++ 运行时 → 系统库](#3-c-运行时--系统库)
  - [3.1. stdlibc++ 策略](#31-stdlibc-策略)
  - [3.2. glibc 到系统调用](#32-glibc-到系统调用)
  - [3.3. 多线程与内存分配](#33-多线程与内存分配)
  - [3.4. 拓展](#34-拓展)
  - [3.5. 性能优化特性](#35-性能优化特性)
    - [3.5.1. 线程本地缓存 (tcache)](#351-线程本地缓存-tcache)
    - [3.5.2. 多种 bins 管理策略](#352-多种-bins-管理策略)
- [4. 系统工具观察](#4-系统工具观察)
- [5. 系统调用进入内核](#5-系统调用进入内核)
  - [5.1. brk/sbrk](#51-brksbrk)
  - [5.2. mmap](#52-mmap)
  - [5.3. 拓展](#53-拓展)
    - [5.3.1. 分配/释放开销](#531-分配释放开销)
    - [5.3.2. 内存碎片处理](#532-内存碎片处理)
  - [5.4. 系统限制和注意事项](#54-系统限制和注意事项)
  - [5.5. 示例说明](#55-示例说明)
  - [5.6. 总结](#56-总结)
- [6. 虚拟内存  → 物理内存](#6-虚拟内存---物理内存)
  - [6.1. 用户态调用brk](#61-用户态调用brk)
  - [6.2. 内核处理brk系统调用](#62-内核处理brk系统调用)
  - [6.3. 缺页中断分配物理内存](#63-缺页中断分配物理内存)
- [7. 物理内存的分配](#7-物理内存的分配)
  - [7.1. 多级页表](#71-多级页表)
  - [7.2. 内核层面的完整流程](#72-内核层面的完整流程)
  - [7.3. 物理内存分配细节](#73-物理内存分配细节)
  - [7.4. 性能优化机制](#74-性能优化机制)
- [8. 总结](#8-总结)


## 1. 整体概括

在C++中，`new`和`delete`是动态内存管理的关键字，它们的底层实现通常依赖于C运行时库（如glibc）提供的`malloc`和`free`函数。在Linux系统下，`malloc`和`free`则进一步通过系统调用来管理内存。主要涉及的系统调用有两个：`brk`（或`sbrk`）和`mmap`。

完整的 new 流程如下

```cpp
// 应用代码
int* p = new int(100);

// 1. 编译器生成代码调用 operator new(sizeof(int))
// 2. operator new 调用 malloc(4)
// 3. glibc 的 malloc:
//    - 检查线程本地缓存 (tcache)
//    - 检查 fast bins（小内存快速分配）
//    - 对于 4 字节，可能在 fast bins 中找到
// 4. 如果 fast bins 没有，检查 small bins
// 5. 如果还没有，调用 brk 扩展堆
// 6. brk 系统调用进入内核
// 7. 内核更新进程的堆指针
// 8. 返回用户态，malloc 管理新内存
// 9. 返回指针，构造函数被调用
// 10. 程序使用该内存时可能触发缺页中断
// 11. 内核分配物理页面并建立页表映射
```

完整的free 流程如下

```cpp
delete p;

// 1. 调用析构函数（对于类对象）
// 2. 调用 operator delete
// 3. operator delete 调用 free
// 4. glibc 的 free:
//    - 对于小内存：放入 fast bins 或 small bins
//    - 对于大内存：调用 munmap
// 5. munmap 系统调用进入内核
// 6. 内核解除虚拟内存映射
// 7. 释放对应的物理页面（可能延迟）
```



## 2. 应用层 → C++ 运行时

调用 `new/delete` 关键字。

```cpp
// 应用代码
MyClass* obj = new MyClass(42);
delete obj;

// 编译器大致转换为：
void* memory = operator new(sizeof(MyClass));
MyClass* obj = static_cast<MyClass*>(memory);
obj->MyClass(42);  // 构造对象

// ... 使用对象 ...

obj->~MyClass();    // 析构对象
operator delete(obj);
```



## 3. C++ 运行时 → 系统库

### 3.1. stdlibc++ 策略

`operator new` 的典型调用链路：

```cpp
operator new() → malloc() → __libc_malloc() → _int_malloc() → 使用 brk 扩展堆或使用 mmap 创建新映射
```

标准库有 glibc 和 libstdC++。glibc 本身是C库，而`operator new`是C++的一部分，它位于libstdc++（GNU的标准C++库）中。

具体的 `operator new`实现如下：

主要文件：

- `libstdc++-v3/libsupc++/new_op.cc` - 全局 operator new
- `libstdc++-v3/libsupc++/new_opnt.cc` - 不抛出异常的 operator new
- `libstdc++-v3/libsupc++/new_opv.cc` - 数组版本的 operator new

全局 operator new（抛出异常版本），通常在 `libstdc++-v3/libsupc++/new_op.cc`：

```cpp
_GLIBCXX_WEAK_DEFINITION void *
operator new (std::size_t sz) _GLIBCXX_THROW (std::bad_alloc)
{
  void *p;

  /* malloc (0) is unpredictable; avoid it.  */
  if (__builtin_expect (sz == 0, false))
    sz = 1;

  while ((p = malloc (sz)) == 0)
    {
      new_handler handler = std::get_new_handler ();
      if (! handler)
	_GLIBCXX_THROW_OR_ABORT(bad_alloc());
      handler ();
    }

  return p;
}
```

不抛出异常的版本，通常在`libstdc++-v3/libsupc++/new_opnt.cc`

```cpp
_GLIBCXX_WEAK_DEFINITION void *
operator new (std::size_t sz, const std::nothrow_t&) noexcept
{
  // _GLIBCXX_RESOLVE_LIB_DEFECTS
  // 206. operator new(size_t, nothrow) may become unlinked to ordinary
  // operator new if ordinary version replaced
  __try
    {
      return ::operator new(sz);
    }
  __catch (...)
    {
      return nullptr;
    }
}
```



### 3.2. glibc 到系统调用

**内存分配器（malloc）的管理策略**：glibc的malloc使用ptmalloc2分配器，它通过一系列链表（bins）来管理已分配和空闲的内存块，以提高分配效率和减少内存碎片。

glibc的`malloc`(基于 ptmalloc2) 在分配内存时，会根据请求的字节数采取不同的策略：

- 对于小内存分配（<=128KB）， 先在 `fastbins` 、`smallbins`中寻找合适的空闲块，若找不到，则使用`brk`系统调用扩展堆空间，然后从新扩展的堆空间中分配内存。brk 用于扩展堆的大小。
- 对于大内存分配（大于等于128KB），使用`mmap`系统调用来分配一块独立的内存映射。

> 注意：这里的128KB是一个默认阈值，可以通过 `mallopt` 函数调整。

默认阈值定义在 `malloc/malloc.c`文件中

```c
#ifndef DEFAULT_MMAP_THRESHOLD
#define DEFAULT_MMAP_THRESHOLD DEFAULT_MMAP_THRESHOLD_MIN
#endif

#ifndef DEFAULT_MMAP_THRESHOLD_MIN
#define DEFAULT_MMAP_THRESHOLD_MIN (128 * 1024)  // 128KB
#endif
```

这样做的目的是为了平衡性能和大内存管理的效率。小内存分配频繁，使用`brk`可以减少系统调用的次数（因为一次`brk`可以扩展堆，然后多次分配都在这个堆上）。而大内存使用`mmap`可以避免堆的碎片化，因为大内存块在释放时可以直接通过`munmap`返还给系统。

```c
// 根据大小选择分配策略
void* malloc(size_t size) {
    if (size < 128 * 1024) {
        // 小内存：使用 brk 管理的堆
        return tcache_allocate(size);  // 线程缓存
    } else {
        // 大内存：直接使用 mmap
        return mmap_allocate(size);
    }
}

void free(void* ptr) {
    if (is_mmap_chunk(ptr)) {
        // mmap 分配的直接 munmap
        munmap(ptr, get_chunk_size(ptr));
    } else {
        // brk 管理的放回相应 bin
        tcache_free(ptr);
    }
}
```

总结一下：

1. 当进程启动后，第一次调用malloc时，堆可能还没有初始化，malloc会通过brk系统调用来扩展堆，然后从堆中分配一块内存给用户。
2. 当用户释放内存时，malloc会将释放的内存块加入到bins（如fastbins）中，以便后续分配时重用。
3. 当再次分配内存时，malloc会先检查bins中是否有合适的空闲块。如果有，就直接使用这个空闲块，而无需调用brk扩展堆。
4. 如果bins中没有合适的空闲块，malloc才会通过brk扩展堆，然后从堆中分配。

所以，对于小于128KB的内存分配，malloc会尝试在bins中查找空闲块，如果找不到，则通过brk扩展堆来分配。而大于128KB的内存分配，则使用mmap系统调用分配一块独立的内存映射区域。





### 3.3. 多线程与内存分配

为了应对多线程场景，glibc 的 `malloc` 引入了 **分配区 (arena)** 的概念，主要是 `main_arena`（主分配区）和 `thread_arena`（非主分配区，也称为线程分配区）。

- **主分配区**：由主线程使用，其内存通常通过 `brk` 系统调用从堆区申请。
- **非主分配区**：当其他线程申请内存，而主分配区正被占用时，系统可能会创建新的非主分配区。这些分配区的内存只能通过 `mmap` 系统调用来申请。每个分配区都有自己的锁，这有助于减少多线程竞争



### 3.4. 拓展

在 glibc 中，`malloc` 的具体实现主要位于 **`malloc/malloc.c`** 文件中。这个文件包含了我们日常使用的 `malloc`、`free`、`realloc` 等内存管理函数的核心代码。

如果你想深入研究 `malloc` 的源码：

- **定位文件**：在 glibc 源码目录下的 `malloc/malloc.c`。
- **核心函数**：重点关注 `__libc_malloc`、`_int_malloc` 等函数。
- **辅助工具**：可以使用 `strace` 跟踪系统调用，或者使用 `mtrace` 等内存调试工具来观察 `malloc` 和 `free` 的行为。



### 3.5. 性能优化特性

#### 3.5.1. 线程本地缓存 (tcache)

```c
// glibc 2.26+ 引入的每线程缓存
typedef struct tcache_perthread_struct
{
    char counts[TCACHE_MAX_BINS];
    tcache_entry *entries[TCACHE_MAX_BINS];
} tcache_perthread_struct;
```



#### 3.5.2. 多种 bins 管理策略

- **Fast bins**: LIFO，单链表，小内存快速分配
- **Small bins**: FIFO，双向链表，精确大小
- **Large bins**: 大小范围，最佳匹配
- **Unsorted bin**: 临时存放释放的块



## 4. 系统工具观察

你可以使用这些工具观察内存分配：

```bash
# 查看内存映射
cat /proc/$$/maps

# 使用 strace 跟踪系统调用
strace -e brk,mmap,munmap ./your_program

# 使用 ltrace 跟踪库调用
ltrace -e malloc,free ./your_program

# 使用 valgrind 分析内存
valgrind --tool=memcheck ./your_program
```





## 5. 系统调用进入内核

在Linux系统下，`malloc`和`free`则进一步通过系统调用来管理内存。主要涉及的系统调用有两个：`brk`（或`sbrk`）和`mmap`。`brk` 和 `mmap` 是 Linux 中两种不同的内存分配机制，它们在用途、特性和性能方面有显著区别。

**传统进程内存布局：**

```
高地址
┌─────────────┐
│   栈 stack   │ ← 向下生长
├─────────────┤
│     ...     │
├─────────────┤
│  内存映射区   │ ← mmap 分配区域
├─────────────┤
│    堆 heap   │ ← 向上生长 (brk)
├─────────────┤
│   BSS 段    │
├─────────────┤
│  数据段 data  │
├─────────────┤
│  代码段 text  │
低地址
```

### 5.1. brk/sbrk

**brk**：是操作系统层面的**堆内存扩展机制**，用于扩展堆空间大小。

`brk`和`sbrk`是用于调整程序堆（heap）顶部的系统调用。在传统的内存布局中，堆是位于程序数据段之后的一块连续内存区域，通过移动堆顶指针来分配或释放内存。

- `brk`：通过传入一个地址来设置堆顶的位置。
- `sbrk`：通过传入一个增量来调整堆顶的位置。

**特点**：

- 用于分配较小块的内存（通常小于128KB）。
- 分配的内存是连续的（在虚拟地址空间中）。
- 频繁分配和释放可能造成内存碎片。
- 只能用于扩展或收缩堆，因此适用于连续的内存分配。
- 只能操作**堆的末尾**。
- 内存只能从低地址向高地址增长。

**工作原理：**

```c
#include <unistd.h>

// brk - 直接设置新的堆结束地址
int brk(void *end_data_segment);

// sbrk - 调整堆的大小并返回之前的堆结束地址
void *sbrk(intptr_t increment);
```

示例：

```c
// 扩展堆 4KB
void* old_brk = sbrk(0);        // 获取当前 brk
void* new_brk = sbrk(4096);     // 扩展堆

// 或者使用 brk
void* current = sbrk(0);
brk(current + 4096);            // 直接设置新 brk
```

底层关系：`sbrk`是基于`brk`实现的。`sbrk`会记录当前的堆结束地址，然后根据增量调用`brk`来设置新的堆结束地址。

```c
void *sbrk(intptr_t increment) {
    void *old_break = sbrk(0);  // 获取当前堆结束地址
    if (brk((char *)old_break + increment) == -1) {
        return (void *)-1;      // 失败
    }
    return old_break;           // 返回之前的地址
}
```



**brk 的应用场景**

```c
// glibc 中小内存分配使用 brk
void* small_malloc(size_t size) {
    if (size < 128 * 1024) {  // 小于128KB
        // 使用 brk 管理的堆内存
        return allocate_from_heap(size);
    }
    // 大内存使用 mmap
    return mmap_alloc(size);
}
```

**总结**：`brk` 和 `sbrk` 是 Linux 系统中管理堆内存的低级系统调用，`sbrk` 更常用于增量式内存分配，而 `brk` 用于精确设置堆边界。在现代编程中，建议使用标准的内存分配函数而不是直接调用这些系统调用。



### 5.2. mmap

`mmap`（内存映射）系统调用可以将文件或设备映射到内存中，也可以用于分配匿名内存（即没有文件背景的内存，用于动态分配）。

**特点**：

- 通常用于分配大块内存（例如，glibc中默认大于128KB的使用`mmap`）。
- 可以在**任意虚拟地址**创建映射，每个映射都是连续的，但分配的内存不一定连续。
- 可以映射文件（用于文件IO）或匿名内存（用于动态分配）。
- 分配和释放通过`mmap`和`munmap`完成，每次分配都会在虚拟地址空间中创建一个新的映射，释放时直接解除映射，因此不会产生碎片问题，但**系统调用开销较大**，每次都有系统调用开销。

**工作原理：**

```c
#include <sys/mman.h>

// 创建内存映射
void *mmap(void *addr, size_t length, int prot, int flags, int fd, off_t offset);

// 解除内存映射
int munmap(void *addr, size_t length);
```

**匿名映射示例（用于内存分配）：**

```c
// 分配 1MB 匿名内存（不关联文件）
void* memory = mmap(NULL,                   // 由系统选择地址
                    1024 * 1024,           // 1MB
                    PROT_READ | PROT_WRITE, // 可读可写
                    MAP_PRIVATE | MAP_ANONYMOUS, // 私有匿名映射
                    -1,                     // 文件描述符（匿名映射用-1）
                    0);                     // 偏移量

// 使用后释放
munmap(memory, 1024 * 1024);
```

**mmap 应用场景**

```c
// 1. 大内存分配
void* large_mem = mmap(NULL, 10 * 1024 * 1024, PROT_READ | PROT_WRITE,
                      MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);

// 2. 文件映射（内存映射文件）
int fd = open("large_file.dat", O_RDONLY);
void* file_mem = mmap(NULL, file_size, PROT_READ, MAP_PRIVATE, fd, 0);

// 3. 共享内存
void* shared_mem = mmap(NULL, size, PROT_READ | PROT_WRITE,
                       MAP_SHARED | MAP_ANONYMOUS, -1, 0);

// 4. 分配栈内存（某些线程实现）
void* thread_stack = mmap(NULL, stack_size, PROT_READ | PROT_WRITE,
                         MAP_PRIVATE | MAP_ANONYMOUS | MAP_STACK, -1, 0);
```

### 5.3. 拓展

#### 5.3.1. 分配/释放开销

```c
// brk 分配 - 开销较小
void brk_allocation() {
    void* start = sbrk(0);
    // 多次小分配可能只涉及一次 brk 调用
    for (int i = 0; i < 1000; i++) {
        // 在已扩展的堆内部分配，无需系统调用
        allocate_from_heap(128);
    }
}

// mmap 分配 - 每次都有系统调用开销
void mmap_allocation() {
    for (int i = 0; i < 1000; i++) {
        // 每次都需要 mmap 系统调用
        mmap(NULL, 4096, PROT_READ | PROT_WRITE, 
             MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    }
}
```



#### 5.3.2. 内存碎片处理

```c
// brk - 容易产生碎片
void brk_fragmentation() {
    void* p1 = malloc(1024);  // 分配块1
    void* p2 = malloc(2048);  // 分配块2  
    void* p3 = malloc(1024);  // 分配块3
    
    free(p2);  // 释放中间块，产生碎片
    // 现在堆布局：[使用][空闲][使用]
    // 后续只能分配 <= 2048 的块到这个空隙
}

// mmap - 无外部碎片
void mmap_no_fragmentation() {
    void* p1 = mmap_alloc(1024);  // 独立映射
    void* p2 = mmap_alloc(2048);  // 独立映射
    void* p3 = mmap_alloc(1024);  // 独立映射
    
    munmap(p2, 2048);  // 完全释放，无碎片
    // 每个映射都是独立的，释放后空间完全可用
}
```



### 5.4. 系统限制和注意事项

brk 的限制

```c
// 堆大小有限制
void check_brk_limits() {
    void* current = sbrk(0);
    void* max_heap = (void*)0x...;  // 系统定义的堆上限
    
    if (current + requested_size > max_heap) {
        // 无法继续扩展堆，需要改用 mmap
        return mmap_alloc(requested_size);
    }
}
```

mmap 的限制

```c
// 虚拟地址空间限制
void check_mmap_limits() {
    // 32位系统：~3GB 用户空间
    // 64位系统：巨大的地址空间
    
    // 映射数量限制
    // cat /proc/sys/vm/max_map_count
}
```



### 5.5. 示例说明

假设我们有一个程序，它使用`new`来分配内存：

```cpp
int* small = new int[100]; // 分配400字节（假设int为4字节），小内存，使用brk
int* large = new int[100000]; // 分配400000字节（约400KB），大内存，使用mmap
```

在底层，第一个分配可能会通过`brk`扩展堆来满足，而第二个分配则会通过`mmap`来分配一块独立的内存区域。

### 5.6. 总结

- `brk`：用于管理堆内存，适用于小内存分配，分配的内存位于堆区，连续且分配效率高，但可能产生碎片。
- `mmap`：用于大内存分配或文件映射，分配的内存位于堆和栈之间的映射区域，每个映射独立，避免碎片，但系统调用开销大。



## 6. 虚拟内存  → 物理内存

`brk` 只是调整了虚拟内存的范围，物理内存的分配是在实际访问内存时通过**缺页中断**来完成的。

处理流程如下：

1. 用户程序调用 new，最终调用brk系统调用（比如通过malloc->sbrk->brk）来扩展堆。
2. 内核处理brk系统调用，更新进程的虚拟内存空间（主要是扩大堆的vm_area_struct结构）。
3. 此时，物理内存并没有立即分配，只是虚拟地址空间扩大了。
4. 当程序访问新分配的内存时（比如写入数据），会发生缺页中断（page fault），因为该虚拟地址还没有对应的物理页。
5. 内核在缺页中断处理程序中分配物理内存页，并建立虚拟地址到物理地址的映射。

下面我们详细分析每一步：

### 6.1. 用户态调用brk

在glibc中，当需要扩展堆时，会调用brk或sbrk。brk系统调用的原型是：

```c
int brk(void *addr);
```

它设置堆的结束地址为addr。sbrk则是相对调整，增加increment字节。

在glibc中，malloc管理堆内存，当当前堆空间不足时，会调用sbrk来扩展堆。注意，现代glibc对于大块内存会使用mmap，这里我们只讨论小块内存使用brk的情况。

### 6.2. 内核处理brk系统调用

当brk系统调用进入内核，会执行`sys_brk`函数（在`mm/mmap.c`中）。内核会检查addr参数是否在合理的范围内（比如不能超过栈，不能超过内存限制等）。然后，内核会调整当前进程的堆 `vm_area_struct`结构，扩展堆区域。

但是，brk系统调用可能只是扩展了虚拟内存区域，并没有实际分配物理内存。内核只是标记这块新扩展的虚拟地址空间为可用的，但并没有建立与物理内存的映射。

### 6.3. 缺页中断分配物理内存

当程序第一次访问新分配的内存时，由于该虚拟地址还没有映射到物理内存，会触发缺页中断。缺页中断处理程序会识别出这个地址位于堆区域，并且是一个合法的地址（在堆的`vm_area_struct`范围内），然后分配一个物理页，并建立页表映射。

注意：brk扩展堆时，可能会扩展一段较大的虚拟地址空间，但是物理内存是按需分配的，即访问到哪一页才分配哪一页。

## 7. 物理内存的分配

在缺页中断处理中，内核会调用伙伴系统分配一个物理页，然后更新页表，使得虚拟地址映射到该物理页。

### 7.1. 多级页表

在扩展堆时，如果虚拟地址空间扩展到了一个新的页表项，内核可能需要分配新的页目录项或页表项。这个过程也是在缺页中断中完成的。



### 7.2. 内核层面的完整流程

用户层调用链

```cpp
// 用户程序
int* arr = new int[1000];  // 分配 4000 字节

// 编译后的近似代码：
void* operator_new_array(size_t size) {
    // 1. 调用 malloc
    void* ptr = malloc(4000);
    if (!ptr) {
        // 处理分配失败，可能调用 new_handler
        std::new_handler handler = std::get_new_handler();
        if (handler) {
            handler();
            return operator_new_array(size);  // 重试
        }
        throw std::bad_alloc();
    }
    return ptr;
}
```

内核层

```c
// 1. malloc -> brk 系统调用
用户态: malloc(4000) -> __libc_malloc -> _int_malloc -> sysmalloc -> brk()

// 2. brk 系统调用进入内核
SYSCALL_DEFINE1(brk) -> do_brk_flags()

// 3. 虚拟内存扩展
do_brk_flags() {
    - 检查参数和权限
    - 页面对齐 (4000 -> 4096)
    - 创建或扩展 VMA
    - 更新进程的 mm_struct
    - 返回新的堆顶地址
}

// 4. 用户程序访问内存
// 当程序实际访问新分配的堆内存时，触发缺页中断
for (int i = 0; i < 1000; i++) {
    arr[i] = i;  // 触发缺页中断
}

// 5. 缺页中断处理
handle_mm_fault() {
    - 遍历页表 (PGD -> P4D -> PUD -> PMD -> PTE)
    - 发现 PTE 不存在
    - 调用 alloc_page() 分配物理页
    - 建立页表映射
    - 返回用户态继续执行
}
```

---

具体函数调用细节

1. 系统调用入口

   ```c
   // 内核系统调用处理
   SYSCALL_DEFINE1(brk, unsigned long, brk)
   {
       struct mm_struct *mm = current->mm;
       unsigned long newbrk, oldbrk;
       
       // 1. 边界检查和权限验证
       if (brk < mm->end_data)
           goto out;
       
       // 2. 页面对齐
       newbrk = PAGE_ALIGN(brk);
       oldbrk = PAGE_ALIGN(mm->brk);
       
       // 3. 如果 brk 没有变化，直接返回
       if (oldbrk == newbrk)
           goto set_brk;
       
       // 4. 处理堆的收缩或扩展
       if (brk <= mm->brk) {
           // 收缩堆 - 释放内存
           if (!do_munmap(mm, newbrk, oldbrk - newbrk, NULL))
               goto set_brk;
       } else {
           // 扩展堆 - 分配新内存
           if (find_vma_intersection(mm, oldbrk, newbrk+PAGE_SIZE))
               goto out;
           
           // 核心：扩展堆内存区域
           if (do_brk_flags(oldbrk, newbrk - oldbrk, 0, NULL) < 0)
               goto out;
       }
   
   set_brk:
       mm->brk = brk;
   out:
       return mm->brk;
   }
   ```

2. 虚拟内存扩展 - do_brk_flags

   ```c
   unsigned long do_brk_flags(unsigned long addr, unsigned long len, 
                             unsigned long flags, struct list_head *uf)
   {
       struct mm_struct *mm = current->mm;
       struct vm_area_struct *vma;
       unsigned long new_addr = addr;
       
       // 1. 参数检查
       len = PAGE_ALIGN(len);
       if (!len)
           return addr;
       
       // 2. 查找或合并相邻的 VMA
       vma = find_vma(mm, addr);
       if (vma && vma->vm_start < addr + len) {
           // 处理重叠情况
           return -ENOMEM;
       }
       
       // 3. 检查虚拟地址空间限制
       if (mm->map_count > sysctl_max_map_count)
           return -ENOMEM;
       
       // 4. 扩展现有的 VMA 或创建新的 VMA
       vma = vma_merge(mm, prev, addr, addr + len, flags,
                      NULL, NULL, pgoff, NULL, NULL_VM_UFFD_CTX);
       if (vma)
           goto out;
       
       // 5. 创建新的 VMA
       vma = vm_area_alloc(mm);
       if (!vma)
           return -ENOMEM;
       
       vma->vm_start = addr;
       vma->vm_end = addr + len;
       vma->vm_flags = flags;
       vma->vm_page_prot = vm_get_page_prot(flags);
       
       // 6. 插入到进程的 VMA 红黑树中
       vma_link(mm, vma, prev, rb_link, rb_parent);
       
   out:
       return new_addr;
   }
   ```

3. 物理内存的按需分配 - 缺页中断

   ```c
   // 缺页中断处理程序
   handle_mm_fault(struct vm_area_struct *vma, unsigned long address,
                   unsigned int flags)
   {
       pgd_t *pgd;
       p4d_t *p4d;
       pud_t *pud;
       pmd_t *pmd;
       pte_t *pte;
       
       // 1. 多级页表遍历
       pgd = pgd_offset(mm, address);
       p4d = p4d_alloc(mm, pgd, address);
       pud = pud_alloc(mm, p4d, address);
       pmd = pmd_alloc(mm, pud, address);
       
       // 2. 处理页表项
       if (pmd_none(*pmd) {
           // 中间页表不存在，需要分配
           if (pmd_alloc_huge(mm, pmd, address))
               return VM_FAULT_OOM;
       }
       
       // 3. 获取页表项
       pte = pte_offset_map(pmd, address);
       
       // 4. 检查页表项状态
       if (!pte_present(*pte)) {
           // 页面不存在，需要分配物理页
           struct page *page;
           
           // 分配零页或新页面
           if (vma->vm_flags & VM_SHARED) {
               page = alloc_page_vma(GFP_HIGHUSER_MOVABLE, vma, address);
           } else {
               // 对于堆内存，通常分配新页面
               page = alloc_zeroed_user_highpage_movable(vma, address);
           }
           
           if (!page)
               return VM_FAULT_OOM;
           
           // 建立页表映射
           entry = mk_pte(page, vma->vm_page_prot);
           set_pte_at(mm, address, pte, entry);
       }
       
       return VM_FAULT_NOPAGE;
   }
   ```

### 7.3. 物理内存分配细节

伙伴系统分配物理页

```c
// 内核物理页分配
struct page *alloc_pages(gfp_t gfp_mask, unsigned int order)
{
    struct page *page;
    
    // 1. 尝试从每CPU页面缓存分配
    page = __alloc_pages_fastpath(gfp_mask, order);
    if (page)
        goto out;
    
    // 2. 慢路径分配
    page = __alloc_pages_slowpath(gfp_mask, order);
    
out:
    // 3. 页面初始化
    if (page) {
        prep_new_page(page, order, gfp_mask);
        // 对于用户页面，需要清零
        if (gfp_mask & __GFP_ZERO)
            clear_highpage(page);
    }
    
    return page;
}
```

页表建立过程

```c
// 建立页表映射
static int __handle_pte_fault(struct vm_fault *vmf)
{
    pte_t entry;
    
    // 1. 检查是否应该使用零页
    if (!vma->vm_ops || !vma->vm_ops->fault) {
        // 匿名映射（如堆内存）
        
        // 2. 分配物理页面
        vmf->page = alloc_page_vma(GFP_HIGHUSER_MOVABLE, vma, vmf->address);
        if (!vmf->page)
            return VM_FAULT_OOM;
        
        // 3. 初始化页面内容为零
        clear_user_highpage(vmf->page, vmf->address);
        
        // 4. 创建页表项
        entry = mk_pte(vmf->page, vma->vm_page_prot);
        entry = pte_mkyoung(entry);
        entry = pte_mkdirty(entry);
        
        // 5. 设置页表项
        set_pte_at(vma->vm_mm, vmf->address, vmf->pte, entry);
    }
    
    return VM_FAULT_NOPAGE;
}
```



### 7.4. 性能优化机制

写时复制 (Copy-on-Write)

```c
// 处理写时复制缺页
static int do_wp_page(struct vm_fault *vmf)
{
    struct page *old_page = vmf->page;
    struct page *new_page;
    
    // 1. 检查是否真的需要复制
    if (page_mapcount(old_page) == 1) {
        // 只有一个映射，直接标记可写
        pte_t entry = pte_mkyoung(pte_mkdirty(vmf->orig_pte));
        set_pte_at(vma->vm_mm, vmf->address, vmf->pte, entry);
        return VM_FAULT_WRITE;
    }
    
    // 2. 需要复制页面
    new_page = alloc_page_vma(GFP_HIGHUSER_MOVABLE, vma, vmf->address);
    if (!new_page)
        return VM_FAULT_OOM;
    
    // 3. 复制页面内容
    copy_user_highpage(new_page, old_page, vmf->address, vma);
    
    // 4. 建立新的页表映射
    pte_t entry = mk_pte(new_page, vma->vm_page_prot);
    entry = pte_mkyoung(pte_mkdirty(entry));
    set_pte_at(vma->vm_mm, vmf->address, vmf->pte, entry);
    
    return VM_FAULT_WRITE;
}
```

透明大页 (Transparent Huge Pages)

```c
// 尝试使用大页处理缺页
static int do_huge_pmd_anonymous_page(struct vm_fault *vmf)
{
    // 检查是否适合使用大页
    if (transparent_hugepage_enabled(vma) &&
        !vma->vm_ops &&
        (vma->vm_flags & VM_HUGEPAGE)) {
        
        // 分配大页 (2MB)
        page = alloc_hugepage_vma(TRANSPARENT_HUGEPAGE_ORDER, vma,
                                 vmf->address, numa_node_id(), 0);
        if (page) {
            // 建立大页映射
            set_huge_pte_at(vma->vm_mm, vmf->address, vmf->pmd,
                           mk_huge_pte(page, vma->vm_page_prot));
            return VM_FAULT_NOPAGE;
        }
    }
    
    // 回退到普通页面
    return do_anonymous_page(vmf);
}
```

垃圾回收机制。当物理内存不足时，内核会触发回收：

```c
// 页面回收
static unsigned long shrink_page_list(struct list_head *page_list,
                                     struct pglist_data *pgdat,
                                     struct scan_control *sc)
{
    LIST_HEAD(ret_pages);
    LIST_HEAD(free_pages);
    
    while (!list_empty(page_list)) {
        struct page *page = lru_to_page(page_list);
        
        // 检查页面状态
        if (PageDirty(page)) {
            // 脏页需要写回
            if (pageout(page, mapping, sc))
                goto activate_page;
        }
        
        if (PageAnon(page)) {
            // 匿名页（如堆内存）可能被交换到swap
            if (!add_to_swap(page))
                goto activate_page;
        }
        
        // 可以回收的页面
        list_move(&page->lru, &free_pages);
    }
    
    // 释放页面
    free_unref_page_list(&free_pages);
    return nr_reclaimed;
}
```

## 8. 总结

从 C++ `new` 到内核物理内存分配的完整流程：

1. **用户层**: `new` → `operator new` → `malloc`
2. **C库层**: `malloc` 管理堆块，必要时调用 `brk`
3. **系统调用**: `brk` 进入内核，扩展虚拟地址空间
4. **内核VMA管理**: 创建/扩展 VMA，更新进程内存描述符
5. **按需分配物理内存**: 程序访问内存时触发缺页中断
6. **物理页分配**: 伙伴系统分配物理页面，建立页表映射
7. **性能优化**: 写时复制、透明大页、页面缓存等机制

```
C++ 代码:
    new MyClass()
        ↓
编译器:
    operator new(sizeof(MyClass))
        ↓
libstdc++:
    malloc(size)
        ↓
glibc malloc:
    __libc_malloc() → _int_malloc() → sysmalloc()
        ↓
分配策略判断:
    if (size < 128KB): 使用 brk 扩展堆
    else: 使用 mmap 创建新映射
        ↓
系统调用:
    brk() 或 mmap()
        ↓
Linux 内核:
    sys_brk() 或 sys_mmap()
        ↓
内核内存管理:
    do_brk() 或 do_mmap()
        ↓
物理内存管理:
    页表操作、缺页异常处理等
```

这个过程体现了现代操作系统内存管理的核心思想：**虚拟内存与物理内存分离**、**按需分配**、**延迟绑定**，既提供了安全隔离，又实现了高效的内存利用。
