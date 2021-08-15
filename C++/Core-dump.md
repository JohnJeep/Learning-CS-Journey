<!--
 * @Author: JohnJeep
 * @Date: 2021-03-18 22:30:09
 * @LastEditTime: 2021-08-15 11:43:15
 * @LastEditors: Windows10
 * @Description: Core dump 的使用描述 
-->

# 1. 什么是Core dump?

Coredump叫做核心转储，它是进程运行时在突然崩溃的那一刻的一个内存快照。操作系统在程序发生异常而异常在进程内部又没有被捕获的情况下，会把进程此刻内存、寄存器状态、运行堆栈等信息转储保存在一个文件里。

   该文件也是二进制文件，可以使用gdb、elfdump、objdump或者windows下的windebug、solaris下的mdb进行打开分析里面的具体内容。



# 2. ulimit
虽然我们知道进程在coredump的时候会产生core文件，但是有时候却发现进程虽然core了，但是我们却找不到core文件。

ulimit  -c 可以设置core文件的大小，如果这个值为0.则不会产生core文件，这个值太小，则core文件也不会产生，因为core文件一般都比较大。

 

使用ulimit  -c unlimited来设置无限大，则任意情况下都会产生core文件。


# 3. 将Core dump生成的文件放在指定的目录
设置 Core Dump 的核心转储文件目录和命名规则

/proc/sys/kernel/core_uses_pid 可以控制产生的 core 文件的文件名中是否添加 pid 作为扩展 ，如果添加则文件内容为 1 ，否则为 0

proc/sys/kernel/core_pattern 可以设置格式化的 core 文件保存位置或文件名 ，比如原来文件内容是 core-%e

可以这样修改 :

echo "/corefile/core-%e-%p-%t" > core_pattern

将会控制所产生的 core 文件会存放到 /corefile 目录下，产生的文件名为 core- 命令名 -pid- 时间戳

以下是参数列表 :

    %p - insert pid into filename 添加 pid

    %u - insert current uid into filename 添加当前 uid

    %g - insert current gid into filename 添加当前 gid

    %s - insert signal that caused the coredump into the filename 添加导致产生 core 的信号

    %t - insert UNIX time that the coredump occurred into filename 添加 core 文件生成时的 unix 时间

    %h - insert hostname where the coredump happened into filename 添加主机名

    %e - insert coredumping executable name into filename 添加命令名


# 4. 打开方式
core文件也是二进制文件，可以使用gdb、readelf、objdump或者windows下的windebug、solaris下的mdb进行打开，分析里面具体内容。









# 5. 参考
- https://zhuanlan.zhihu.com/p/98700797
- https://blog.csdn.net/sunxiaopengsun/article/details/72974548
- https://blog.csdn.net/p942005405/article/details/102059719
- [Linux Core Dump](https://www.cnblogs.com/hazir/p/linxu_core_dump.html)
