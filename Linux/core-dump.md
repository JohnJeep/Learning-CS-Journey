<!--
 * @Author: JohnJeep
 * @Date: 2021-03-18 22:30:09
 * @LastEditTime: 2022-01-27 15:42:15
 * @LastEditors: DESKTOP-0S33AUT
 * @Description: Core dump 使用 
-->

# 1. 什么是 Core dump?

Core dump 中文翻译为“核心转储”，它是进程运行时在突然崩溃的那一刻的一个内存快照。

程序 core 是指应用程序无法保持正常 running 状态而发生的崩溃行为。程序 core 时会生成相关的 core-dump 文件，是程序崩溃时程序状态的数据备份。

操作系统在程序发生异常而异常在进程内部又没有被捕获的情况下，会把内存、处理器、寄存器、程序计数器、栈指针等状态信息保存在一个文件里。该文件是二进制文件，使用 gdb、elfdump、objdump 或者 windows 下的 windebug、solaris下 的 mdb 等工具打开和分析文件的内容。

# 2. core 产生原因

## 2.1. Core dump 产生前提条件

进程在 `core dump` 的时候会产生 core 文件，但是有时候却发现进程虽然发生了 core dump，但是在体统中却找不到 core文件。那到底是哪儿出了问题？

`ulimit  -c `可以设置 core 文件的大小，如果这个值为 0，则不会产生 core 文件，这个值太小，则 core 文件也不会产生，因为core文件一般都比较大。

使用 `ulimit  -c unlimited` 来设置无限大，则任意情况下都会产生 core 文件。

## 2.2. Core dump 生成目录

设置 Core dump 的核心转储文件目录和命名规则

`/proc/sys/kernel/core_uses_pid `可以控制产生的 core 文件的文件名中是否添加 pid 作为扩展 ，如果添加则文件内容为 1 ，否则为 0

`proc/sys/kernel/core_pattern` 可以设置格式化的 core 文件保存位置或文件名 ，比如原来文件内容是 core-%e

可以这样修改 :

```bash
echo "/corefile/core-%e-%p-%t" > core_pattern
```

将产生的 core 文件存放到` /corefile` 目录下，产生的文件名为 `core- 命令名 -pid- 时间戳`

以下是参数列表 

```bash
%p - insert pid into filename 添加 pid
%u - insert current uid into filename 添加当前 uid
%g - insert current gid into filename 添加当前 gid
%s - insert signal that caused the coredump into the filename 添加导致产生 core 的信号
%t - insert UNIX time that the coredump occurred into filename 添加 core 文件生成时的 unix 时间
%h - insert hostname where the coredump happened into filename 添加主机名
%e - insert coredumping executable name into filename 添加命令名
```

# ④修改core文件保存路径

●默认生成的core文件保存在可执行文件所在的目录下，文件名为core。

●通过修改/proc/sys/kernel/core_uses_pid文件使生成的core文件加上pid号，echo 1>/proc/sys/kernel/core_uses_pid。

● 还可以通过修改/proc/sys/kernel/core_pattern控制生成的core文件保存的位置以及文件名。

echo "/home/core-%e-%p-%t" > /proc/sys/kernel/core_pattern




# 3. GDB定位 Core 文件

定位core的基本流程可总结为以下几步：

1. 明确core的大致触发原因。机器问题？自身程序问题？
2. 定位代码行。哪一行代码出现了问题。
3. 定位执行指令。哪一行指令干了什么事。
4. 定位异常变量。指令不会有问题，是指令操作的变量不符合预期。

善于利用**汇编指令**以及 **打印指令（x、print、display**可以更有效的定位Core。



利用 GDB 调试生成的 core 文件。

```bash
# gdb execulte_file core_file
gdb hello-word.out core.12259
```

# 4. 参考

- https://zhuanlan.zhihu.com/p/98700797
- https://blog.csdn.net/sunxiaopengsun/article/details/72974548
- https://blog.csdn.net/p942005405/article/details/102059719
- [Linux Core Dump](https://www.cnblogs.com/hazir/p/linxu_core_dump.html)
- [如何快速定位程序Core？](https://mp.weixin.qq.com/s?__biz=MzkyODE5NjU2Mw==&mid=2247488668&idx=1&sn=ea69b41c8215d1a3aca8ed20970e9eea&chksm=c21d2620f56aaf362971e7de0abb79feb8d9863e683e9b8d66eb78c6342ca557fa3ac6f24a43&mpshare=1&scene=24&srcid=0814CXcDIJFXqn3sYEBmN0jw&sharer_sharetime=1628906496373&sharer_shareid=1813da429599d3785585eac965f1aa77#rd)
