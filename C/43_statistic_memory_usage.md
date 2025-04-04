## 背景

在程序中插入自定义的统计代码，来记录内存的使用情况。通过在关键位置调用操作系统提供的内存使用统计函数，如`getrusage()`或`mallinfo()`等，可以获取程序的内存使用情况，并将其记录下来。这样你就可以在程序运行过程中实时监测内存的使用情况。

`getrusage()`和`mallinfo()`是两个不同的函数，用于获取不同层面的内存使用情况。

## getrusage()

`getrusage()`函数用于获取系统级别的资源使用情况，包括进程的总体内存使用情况以及其他资源的统计信息。它返回的结构体`struct rusage`包含了多个字段，可以获取最大常驻内存集、页面错误次数等信息。这些信息是从操作系统的角度统计的，适用于整个进程的内存使用情况。

```c
// 头文件
#include <sys/time.h>
#include <sys/resource.h>

int getrusage(int who, struct rusage *usage);

// 结构体
struct rusage {
  struct timeval ru_utime; /* user CPU time used */
  struct timeval ru_stime; /* system CPU time used */
  long   ru_maxrss;        /* maximum resident set size */
  long   ru_ixrss;         /* integral shared memory size */
  long   ru_idrss;         /* integral unshared data size */
  long   ru_isrss;         /* integral unshared stack size */
  long   ru_minflt;        /* page reclaims (soft page faults) */
  long   ru_majflt;        /* page faults (hard page faults) */
  long   ru_nswap;         /* swaps */
  long   ru_inblock;       /* block input operations */
  long   ru_oublock;       /* block output operations */
  long   ru_msgsnd;        /* IPC messages sent */
  long   ru_msgrcv;        /* IPC messages received */
  long   ru_nsignals;      /* signals received */
  long   ru_nvcsw;         /* voluntary context switches */
  long   ru_nivcsw;        /* involuntary context switches */
};
```

示例

```c
#include <iostream>
#include <sys/resource.h>

// 获取当前进程的内存使用情况，并打印相关信息
void PrintMemoryUsage()
{
    struct rusage usage;
    if (getrusage(RUSAGE_SELF, &usage) == 0)
    {
        std::cout << "内存使用情况：" << std::endl;
        std::cout << "最大常驻内存集：" << usage.ru_maxrss << " KB" << std::endl;
        std::cout << "页面错误次数：" << usage.ru_majflt << std::endl;
        std::cout << "不可恢复的页面错误次数：" << usage.ru_minflt << std::endl;
    }
    else
    {
        std::cerr << "无法获取内存使用情况" << std::endl;
    }
}

int main()
{
    // 在合适的位置调用 PrintMemoryUsage() 来记录内存使用情况

    // 示例：打印程序开始时的内存使用情况
    PrintMemoryUsage();

    // 在这里添加你的代码

    // 示例：打印程序结束时的内存使用情况
    PrintMemoryUsage();

    return 0;
}
```

## mallinfo()

`mallinfo()`函数是GNU C库提供的一个函数，用于获取堆内存的使用情况。它返回的结构体`struct mallinfo`包含了堆内存的统计信息，如总分配空间、空闲空间、空闲块数量等。这些信息是从堆内存管理器的角度统计的，适用于特定的堆内存使用情况。

```c
// 头文件
#include <malloc.h>
struct mallinfo mallinfo(void);

// 结构体
struct mallinfo {
  int arena;     /* Non-mmapped space allocated (bytes) */
  int ordblks;   /* Number of free chunks */
  int smblks;    /* Number of free fastbin blocks */
  int hblks;     /* Number of mmapped regions */
  int hblkhd;    /* Space allocated in mmapped regions (bytes) */
  int usmblks;   /* Maximum total allocated space (bytes) */
  int fsmblks;   /* Space in freed fastbin blocks (bytes) */
  int uordblks;  /* Total allocated space (bytes) */
  int fordblks;  /* Total free space (bytes) */
  int keepcost;  /* Top-most, releasable space (bytes) */
};
```

实例：

```c
#include <iostream>
#include <malloc.h>

int main()
{
    // 调用 mallinfo() 函数获取堆内存的使用情况
    struct mallinfo info = mallinfo();

    std::cout << "总分配空间：" << info.arena << " bytes" << std::endl;
    std::cout << "空闲空间：" << info.fordblks << " bytes" << std::endl;
    std::cout << "空闲块数量：" << info.ordblks << std::endl;
    // 其他字段...

    return 0;
}
```

## 用法总结

所以，`getrusage()`和`mallinfo()`函数提供了不同层面和不同粒度的内存使用统计信息。具体选择哪个函数取决于你想要获取的内存使用情况的精度和范围。

如果你想获取整个进程的内存使用情况，包括堆内存以外的资源使用情况，例如最大常驻内存集和页面错误次数等，那么`getrusage()`函数是一个更合适的选择。

如果你主要关注堆内存的使用情况，例如总分配空间、空闲空间等，那么`mallinfo()`函数可能更适合你的需求。

需要注意的是，`mallinfo()`函数是GNU C库特有的函数，在某些平台上可能不可用。另外，它仅适用于使用GNU C库的程序。如果你的程序不使用GNU C库，那么`mallinfo()`函数将不可用。在这种情况下，你可以考虑使用其他方法来获取堆内存的使用情况，如使用第三方内存分析工具或自定义统计代码。

