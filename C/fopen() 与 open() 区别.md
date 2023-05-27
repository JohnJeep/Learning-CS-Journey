<!--
 * @Author: JohnJeep
 * @Date: 2022-03-09 17:36:33
 * @LastEditors: JohnJeep
 * @LastEditTime: 2023-05-27 17:04:47
 * @Description:  fopen 函数与 open 函数的区别
 * Copyright (c) 2023 by John Jeep, All Rights Reserved. 
-->

## open() 函数

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

## fopen() 函数

`fopen()` 是 C 语言中的库函数，在不同的操作系统中调用不同的内核 `API`， 返回的是一个指向 `FILE` 结构体的指针。

Linux 中实底层源码中的 `FILE` 结构体是 `_IO_FILE` 结构体的别名，其源码为：`typedef struct _IO_FILE FILE;`，最终调用的是 Linux 操作系统的 API 函数 `open`。其 `_IO_FILE` 结构体源码

```cpp
struct _IO_FILE {
  int _flags;   /* High-order word is _IO_MAGIC; rest is flags. */
#define _IO_file_flags _flags

  /* The following pointers correspond to the C++ streambuf protocol. */
  /* Note:  Tk uses the _IO_read_ptr and _IO_read_end fields directly. */
  char* _IO_read_ptr; /* Current read pointer */
  char* _IO_read_end; /* End of get area. */
  char* _IO_read_base;  /* Start of putback+get area. */
  char* _IO_write_base; /* Start of put area. */
  char* _IO_write_ptr;  /* Current put pointer. */
  char* _IO_write_end;  /* End of put area. */
  char* _IO_buf_base; /* Start of reserve area. */
  char* _IO_buf_end;  /* End of reserve area. */
  /* The following fields are used to support backing up and undo. */
  char *_IO_save_base; /* Pointer to start of non-current get area. */
  char *_IO_backup_base;  /* Pointer to first valid character of backup area */
  char *_IO_save_end; /* Pointer to end of non-current get area. */

  struct _IO_marker *_markers;

  struct _IO_FILE *_chain;

  int _fileno;
#if 0
  int _blksize;
#else
  int _flags2;
#endif
  _IO_off_t _old_offset; /* This used to be _offset but it's too small.  */

#define __HAVE_COLUMN /* temporary */
  /* 1+column number of pbase(); 0 is unknown. */
  unsigned short _cur_column;
  signed char _vtable_offset;
  char _shortbuf[1];

  /*  char* _save_gptr;  char* _save_egptr; */

  _IO_lock_t *_lock;
#ifdef _IO_USE_OLD_IO_FILE
};
```

而 windows 中实底层源码中的 `FILE` 结构体是 `_iobuf`，最终调用的是 Windows 操作系统的 API 函数 `CreateFile`。其 `_iobuf` 源码为

```cpp
typedef struct _iobuf
{
    void* _Placeholder;
} FILE;
```

fopen 函数 API 接口使用说明

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

示例

```c
#include <stdio.h>
#include <stdlib.h>

int main(void)
{
    FILE* fp = fopen("test.txt", "r");
    if(!fp) {
        perror("File opening failed");
        return EXIT_FAILURE;
    }

    int c; // 注意：int，非char，要求处理EOF
    while ((c = fgetc(fp)) != EOF) { // 标准C I/O读取文件循环
       putchar(c);
    }

    if (ferror(fp)) {
        puts("I/O error when reading");
    }
    else if (feof(fp)) {
        puts("End of file reached successfully");
    }
    fclose(fp);
}
```

## fopen() 与 open() 区别

1. Windows 下 `CreateFile` 可以通过参数来制定，保证读写是否线程安全，而 `fopen` 则不可以。
2. `fopen` 工作在用户空间，`open` 是 Linux 的系统调用，所以工作在内核空间。

不同的进程同时访问一个文件，给文件加锁是有效的；而一个进程中的多个线程或协程同时对同一个文件进行加锁会互相覆盖掉，是无效的。


## Reference

- [Windows API](https://zh.wikipedia.org/wiki/Windows_API)
- [关于C++：为什么std::fstreams这么慢？ | 码农家园](https://www.codenong.com/26095160/)
- [C/C++读写文件的几种方法fstream fopen、fwrite()、fread()操作](https://www.cnblogs.com/ZY-Dream/p/11181924.html)