fopen() 与 open() 区别



open() 函数

open()是一个系统调用函数,用来打开或创建一个文件，通过不同的 flags 选项实现不同功能。

```c
// 头文件
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

// 函数原型
int open(const char *pathname, int flags);
int open(const char *pathname, int flags, mode_t mode);

```



fopen() 函数

`fopen()` 是 C 语言中的库函数，在不同的操作系统中调用不同的内核 `API`， 返回的是一个指向 `FILE` 结构体的指针。其实底层源码中 `FILE` 结构体是 `_IO_FILE` 结构体的别名，源码为：`typedef struct _IO_FILE FILE;`

```cpp
// 头文件
#include <stdio.h>

// 函数原型
FILE *fopen(const char *path, const char *mode);

// 返回值
成功：获取文件信息，包括文件名、文件状态、当前读写位置等，并将这些信息保存到一个 FILE 类型的结构体变量中，然后将该变量的地址返回。
错误：返回 NULL 或 errno
```

调用 fopen() 函数时必须指明读写权限，但是可以不指明读写方式（此时默认为`"t"`）。

文件打开方式由 r、w、a、t、b、+ 六个字符拼成，各字符的含义是：

- r(read)：读
- w(write)：写
- a(append)：追加
- t(text)：文本文件
- b(binary)：二进制文件
- +：读和写