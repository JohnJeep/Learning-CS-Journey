<!--
 * @Author: JohnJeep
 * @Date: 2020-05-21 19:19:20
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-11-25 21:41:20
 * @Description: GCC Usage
 * Copyright (c) 2025 by John Jeep, All Rights Reserved. 
-->

- [1. Introduction](#1-introduction)
- [2. 源码编译三部曲](#2-源码编译三部曲)
- [3. CentOS7 安装高版本 gcc8/g++8](#3-centos7-安装高版本-gcc8g8)
- [4. 拓展知识点](#4-拓展知识点)


## 1. Introduction

GCC 原名为 GNU C 语言编译器（GNU C Compiler），只能处理 C 语言。但其很快扩展，变得可处理 C++，后来又扩展为能够支持更多编程语言，如 Fortran、Pascal、Objective -C、Java、Ada、Go 以及各类处理器架构上的汇编语言等，所以改名GNU编译器套件（GNU Compiler Collection）

```bash
常用参数项
-g(gdb)                生成调试信息
-Wall                  编译时生成调试信息
-E(prEprocessed)       源文件文件 .c 生成 预处理文件 .i
-S(aSsembler)          预处理文件 .i 生成汇编文件 .s
-c(compile小写)         汇编文件 .s 生成可执行文件 .o 
-o(output 小写)         生成可执行的二进制文件(类似于Windows中的.exe文件)
-L(link)                链接库路径
-O(Optimizations 大写)  优化代码

-I(dIr)                 指定include头文件
  gcc test.c -I ./include -o test.out  使用 -I 链接指定目录下(./include)的头文件进行编译生成可执行文件。

-D(Defn macro)           指定相关的宏文件(控制日志log输出)
  链接指定目录下(./include)的头文件进行编译生成可执行文件，并使用 -D 链接定义的 DEBUG 宏，生成调试信息。
  gcc test.c -I ./include -o test.out -D DEBUG  

-fPIC(position independent code)  生成与位置无关的代码
 
--Woverride-virtual   编译时检查虚函数


```

- [GCC 包下载：fedoraproject.org](https://archives.fedoraproject.org/pub/)
- [gun.org](http://ftp.gnu.org/gnu/)
- [清华大学 GNU 源镜像](https://mirrors.tuna.tsinghua.edu.cn/gnu/)

## 2. 源码编译三部曲

第一步：执行脚本 configure 文件，设置指定的参数，建立 Makefile 文件。

```sh
./configure --prefix=指定软件路径
例如：../configure --prefix=/usr/local/gcc-4.9.4 -enable-checking=release -enable-languages=c,c++ -disable-multilib

CentOS7 4.8.4 默认安装时的配置
../configure --prefix=/usr --mandir=/usr/share/man --infodir=/usr/share/info --with-bugurl=http://bugzilla.redhat.com/bugzilla --enable-bootstrap --enable-shared --enable-threads=posix --enable-checking=release --with-system-zlib --enable-__cxa_atexit --disable-libunwind-exceptions --enable-gnu-unique-object --enable-linker-build-id --with-linker-hash-style=gnu --enable-languages=c,c++,objc,obj-c++,java,fortran,ada,go,lto --enable-plugin --enable-initfini-array --disable-libgcj --with-isl=/builddir/build/BUILD/gcc-4.8.5-20150702/obj-x86_64-redhat-linux/isl-install --with-cloog=/builddir/build/BUILD/gcc-4.8.5-20150702/obj-x86_64-redhat-linux/cloog-install --enable-gnu-indirect-function --with-tune=generic --with-arch_32=x86-64 --build=x86_64-redhat-linux

参数项
  --prefix：指定安装路径。
  --enable-threads=posix：启用POSIX标准的线程支持。要让程序能在符合POSIX规范的linux发布版上正确运行，就应该启用该选项。这里取决于目标操作系统的类型，其它可用值有：aix、dec、solaris、win32等。
  --disable-checking：不对编译时生成的代码进行一致性检查（检查的话一般设置为：--enable-checking=release）。建议机器硬件配置较低以及不愿等待太久编译时间的童鞋，可以设置为disable，但是这会增加产生未预期的错误的风险。
  --disable-multilib：如果你的操作系统是32位，默认就已经设置为disable，这意味着gcc仅能生成32位的可执行程序。如果你的操作系统是64位，默认设置为enable，这意味着用gcc编译其它源文件时可以通过-m32选项来决定是否生成32位机器代码。由于我们这里是64位系统上，所以要禁止生成32位代码。
  --enable-languages=c,c++：支持的高级语言类型和运行时库，可以设置的所有语言还包括ada、Fortran、java、objc、obj-c++、GO等语言。这里只开启了c和c++，因为支持的语言越多，就需要安装越多的相应静态与动态库，等待的时间也越久。
```

第二部：执行 make 命令

```
执行 make 命令进行 编译。
```

第三步：执行 make install

```
安装软件到第一步 ./configure 后面指定的路径下。
```

## 3. CentOS7 安装高版本 gcc8/g++8

```sh
1、安装软件仓库包 scl: yum install centos-release-scl
2、安装 gcc/g++，数字 8 对应的是 gcc/g++8: yum install devtoolset-8-gcc devtoolset-8-gcc-c++
3、shell 终端临时设置默认版本，重启后失效: scl enable devtoolset-8 -- bash；
长期有效设置：vim /etc/profile 文件中的最后一行加入: source /opt/rh/devtoolset-8/enable
```

高版本 GCC 编译器编译 C++11 之下的代码，可能出现的问题？

许多 c++11 功能都需要 C++ 标准库的新 libc++ 实现。但是 libc++ 与旧的 libstdc++ 不兼容，但目前大多数软件通常都与旧的 libstdc++ 链接。

```
libc++ 使用内联 namespace 来帮助确保 ABI不兼容类型不会被误认为是彼此之间的错误。如果接口(interface) 直接使用 libc++ std::string，则期望 libstdc++ std::string 的库将不会链接到该接口(interface)，因为实际的符号是不同的 :std::string 与 std::__1::string。
```

## 4. 拓展知识点

```sh
GCC4.8.1 支持 C++11
  GCC 4.8.1 will be C++11 feature-complete [2013-04-01]
Support for C++11 ref-qualifiers was added to the GCC 4.8 branch, making G++ the first C++ compiler to implement all the major language features of the C++11 standard. This functionality will be available in GCC 4.8.1.
```

```sh
GCC5.3 支持 C++14
  GCC 5 C++14 language feature-complete [2014-12-23]
  Support for all C++14 language features has been added to the development sources for GCC, and will be available when GCC 5 is released next year. Contributed by Jason Merrill, Braden Obrzut, Adam Butcher, Edward Smith-Rowland, and Jakub Jelinek.
```

- [GNU GCC 历史版本发布说明](https://gcc.gnu.org/news.html)
- [离线 GCC 安装教程](https://cloud.tencent.com/developer/article/1176706)
- [Linux编译安装GNU gcc 4.9.4](https://blog.csdn.net/dhy012345/article/details/89642421)
- [【推荐】CentOS安装gcc-4.9.4+更新环境+更新动态库)](https://www.cnblogs.com/brishenzhou/p/8820237.html)
- [**CentOS下离线安装gcc环境，图文详细，方法全面**](https://blog.51cto.com/u_14089205/2485301?u_atoken=0475e265-abfb-432c-bfb1-c97021d375cd&u_asession=01ogsxgS7WqDDhfHxJBA3tySuJoQLPU7Mo1t0qQM8oX1gIjn9NkTPKsfuM9wUeCkZyX0KNBwm7Lovlpxjd_P_q4JsKWYrT3W_NKPr8w6oU7K9T6Skf8G7F5_odZ0I0AeJU88K6FZziwTt7TNrPVU-d_2BkFo3NEHBv0PZUm6pbxQU&u_asig=05NjW6cy-7_DOVLCJ5yY0xUjyZdNii222gZ8CBYNC2-kxLwqNZcRznzNJlTGFikFQNdDeTXXaCH9gxxB_bXfuDd8FLqKsnQLHkUQ1ZHfh3CQ-AEuPGBzKGc0px5fa5PyZE0WZX_farO5ZSDrFzcwrvDd4ss8s7GG-KUqB6plMu8BP9JS7q8ZD7Xtz2Ly-b0kmuyAKRFSVJkkdwVUnyHAIJzUGHsagEujBC_AvKS2HjtZaUdPfTTV87rFMrUrrgd0Lqa8KPmxEvUknfKsop6MC1kO3h9VXwMyh6PgyDIVSG1W9Vksf1-kHx6lS1DDW9Qv6U4c0g9zEGGygqGkD8zIaRcgyMCcOAeA64HiKzlA8DugWt59etCjmgwcMzWzJCS1N4mWspDxyAEEo4kbsryBKb9Q&u_aref=3kSTCuGS3qx%2FcOMR4cQo%2FTU%2B7iE%3D)