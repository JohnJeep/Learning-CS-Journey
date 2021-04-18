<!--
 * @Author: JohnJeep
 * @Date: 2020-04-23 20:37:04
 * @LastEditTime: 2021-04-16 23:03:12
 * @LastEditors: Please set LastEditors
 * @Description: GDB、Makefile使用剖析
--> 

<!-- TOC -->

- [1. GDB调试](#1-gdb调试)
- [2. 启动GDB打普通断点](#2-启动gdb打普通断点)
- [3. 调试程序](#3-调试程序)
- [4. TUI模式](#4-tui模式)
- [5. 调试多进程](#5-调试多进程)
- [6. 调试多线程](#6-调试多线程)
- [7. 查看内存](#7-查看内存)
- [8. Hardware watchpoint(硬件断点)](#8-hardware-watchpoint硬件断点)
- [9. 底层原理](#9-底层原理)
- [10. 参考](#10-参考)

<!-- /TOC -->

# 1. GDB调试






# 2. 启动GDB打普通断点
> 在进行GDB调试之前需要先打断点。GDB中的断点分为：普通断点、观察断点、捕捉断点，一般使用 `break` 打的断点称为普通断点，使用 `watch` 打的断点称为硬件断点，使用 `catch` 打的断点称为捕捉断点。


- `gcc xxx.c -g -o xxx.out` 使用 gdb 调试程序之前,必须使用 `-g` 或 `–ggdb`编译选项编译源文件。
- `gdb a.out` 启动GDB，其中 `a.out` 是带有调试信息的可执行文件
- `list(l)`  默认查看`10行` 程序代码
- `enter键` 执行上一次输入过的命令
- `l xxx.c: zz(或行号)` 查看 `xxx.c` 文件中的 `zz` 函数
- `break(b) main(或行号)`   在main函数处设置断点。  `break 20`   在 `20行` 处设置断点。
- `b 22 if i==10`  设置条件断点。在22行处，当 `i==10` 时设置一个断点。
  > 注意: 有循环体的时候，断点应设置在循环体的内部。在循环体(for、while等语句)的时候程序不会停。
- `info(i) break`  查看设置的断点信息内容
- 启动GDB
  - `start` 程序只运行一次就停止了
  - `run(r)`



# 3. 调试程序
  - `next(n)    ` 单步执行程序，但不进入子函数内部
  - `u` 跳出单次循环，然后跳到循环后面的语句。
  - `step(s)    ` 单步执行程序，进入子函数内部
  - `finish` 从函数体内部跳出去。如果该函数体内部打的有断点，首先需要把断点删除，然后再跳出函数体。
  - `continue(c)` 继续执行程序
  - `print(p) 变量名` 查看指定变量值
  - `ptype 变量名` 查看变量的类型
  - `display 变量名` 在循环的时候实时追踪变量的值。 `display i` 追踪变量 `i` 的值
  - `undisplay 变量名的编号`  不追踪某个变量的值。首先查看不需要追踪变量的编号 `i(info) display` ，然后使用 `undisplay 变量名的编号` 去掉不用追踪的变量。
  - `del(d) 断点编号N` 删除当前编号为 N 的断点
  - `set var=value` 设置变量的值
  - `quit(q)` 退出gdb
  - `ni` 单步执行汇编指令，不进入子函数内部
  - `si` 单步执行汇编指令，进入子函数内部

# 4. TUI模式 
- TUI模式下，总共有4种窗口
  - 命令窗口
  - 源码窗口：可以使用 `PageUp`，`PageDown` 和4个方向键来查看源码。
  - 汇编窗口
  - 寄存器窗口
- `ctrl + x + a` 进入调试图形界面，再按同样的快捷键，退出调试图形化窗口。
- `ctrl + x + 1` 进入汇编调试图形界面，再按一次退出汇编调试图形界面。
- `ctrl + x + 2` 显示其中2个窗口，数字 `2` 表示除了显示命令窗口外还可以同时再显示2个窗口。连续按下 `Ctrl+x+2` 就会出现这三个窗口的两两组合。
- TUI模式下有时显示会出现混叠现象，此时按下 `Ctrl+l`（是小写的L）进行刷新。
- 断点显示的几种状态
  - `B` 表示断点处代码已经运行至少一次
  - `b` 表示断点处代码还没有运行到
  - `+` 表示该断点处于enable状态
  - `-` 表示该断点处于disable状态


# 5. 调试多进程
- gdb追踪多个分支（父子进程）
  - `set follow-fork-mode child`  追踪子进程
  - `set follow-fork-mode parent` 追踪父进程

# 6. 调试多线程
`attach 线程号`

# 7. 查看内存
- `back trace(bt)` 打印当前函数调用栈的所有信息
- `examine(x)` 查看内存地址中的值
  - `n` 是一个正整数，表示显示内存的长度，也就是说从当前地址向后显示几个地址的内容。 
  - `f` 按浮点数格式显示变量。  
  - `u` 从当前地址往后请求的字节数，如果不指定的话，GDB默认是4个bytes。b表示单字节，h表示双字节，w表示四字节，g表示八字节。

# 8. Hardware watchpoint(硬件断点)
普通断点：需要程序运行到哪行，你就在哪行设置断点，然后等程序运行到断点处可以单步执行，查看内存变量，遇到多个位置修改同一个变量时，并且要查看是谁改变了变量的时候，就要设置多个断点来进行查看。

观硬件断点也叫观察断点。观察断点就是为了要监控某个变量或者表达式的值，通过值的变化情况来判断程序的执行过程是否存在异常或者Bug。只有某个变量或者表达式的值发生了改变，程序才会停止执行。相比普通断点，观察断点不需要我们预测变量（表达式）值发生改变的具体位置。

用法 
```
(gdb) watch var_name
```
> 在使用 `watch var_name` 命名之前，需要使用 `file a.out`，加载 a.out 中的 `symbol table`，只有符号表加载成功后，才能打硬件断点。
![](./pictures/watch-break.png)

```c
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>

int g_var = 0;

void* thread_func(void* args)
{
    sleep(5);

    g_var = 1;
}

int main()
{
    int i = 0;
    pthread_t tid = 0;

    pthread_create(&tid, NULL, thread_func, NULL);

    for(i=0; i<10; i++)
    {
        printf("g_var = %d, tid = %ld\n", g_var, tid);

        sleep(1);
    }
}

```


# 9. 底层原理
- ptrace系统函数是Linux内核提供的一个用于进程跟踪的系统调用，通过它，一个进程(gdb)可以读写另外一个进程(test)的指令空间、数据空间、堆栈和寄存器的值。而且gdb进程接管了test进程的所有信号，也就是说系统向test进程发送的所有信号，都被gdb进程接收到，这样一来，test进程的执行就被gdb控制了，从而达到调试的目的。

- gdb底层的调试机制是怎样的？
  > 系统首先会启动gdb进程，这个进程会调用系统函数fork()来创建一个子进程，这个子进程做两件事情：
  > 1. 调用系统函数ptrace；
  > 2. 通过execc来加载、执行可执行程序 test，那么test程序就在这个子进程中开始执行了。
<img src="./pictures/gdb-ptrace.png">



# 10. 参考
- [用图文带你彻底弄懂GDB调试原理](https://mp.weixin.qq.com/s?__biz=MzA3MzAwODYyNQ==&mid=2247483895&idx=1&sn=ba35d1823c259a959b72a310e0a92068&scene=21#wechat_redirect)
- [100个gdb小技巧](https://wizardforcel.gitbooks.io/100-gdb-tips/content/set-watchpoint.html)
- [线程的查看以及利用gdb调试多线程](https://blog.csdn.net/zhangye3017/article/details/80382496)
- [YouTube: MyGeekAdventures 临场演示如何使用GBD去调试代码](https://www.youtube.com/watch?v=xQ0ONbt-qPs&list=PL5Py8jKS3yHOco9op3r_6JN2IKmopTt7s) <font color=red>： 重点看 </font>

