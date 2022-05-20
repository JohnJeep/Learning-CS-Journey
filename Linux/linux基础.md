<!--

 * @Author: JohnJeep
 * @Date: 2020-04-04 09:46:51
 * @LastEditTime: 2022-04-07 18:09:29
 * @LastEditors: DESKTOP-0S33AUT
 * @Description: Linux 基础用法笔记
--> 

<!-- TOC -->

- [1. Linux Basic](#1-linux-basic)
- [2. 硬件基础知识](#2-硬件基础知识)
  - [2.1. MBR](#21-mbr)
  - [2.2. GPT](#22-gpt)
  - [2.3. BIOS与UEFI](#23-bios与uefi)
  - [2.4. 分区](#24-分区)
- [3. 基础命令](#3-基础命令)
  - [3.1. touch](#31-touch)
  - [3.2. rm](#32-rm)
  - [3.3. mkdir](#33-mkdir)
  - [3.4. rmdir](#34-rmdir)
  - [3.5. mv](#35-mv)
  - [3.6. cp](#36-cp)
  - [3.7. ls](#37-ls)
  - [3.8. stat](#38-stat)
  - [3.9. disk](#39-disk)
  - [3.10. fdisk](#310-fdisk)
  - [3.11. df](#311-df)
  - [3.12. mount](#312-mount)
  - [3.13. umount](#313-umount)
  - [3.14. which](#314-which)
  - [3.15. whereis](#315-whereis)
  - [3.16. hostname](#316-hostname)
- [4. 系统相关](#4-系统相关)
- [5. 文件查看命令](#5-文件查看命令)
  - [5.1. cat](#51-cat)
  - [5.2. tac](#52-tac)
  - [5.3. nl](#53-nl)
  - [5.4. more](#54-more)
  - [5.5. less](#55-less)
  - [5.6. od](#56-od)
  - [5.7. head 和 tail](#57-head-和-tail)
  - [5.8. uniq](#58-uniq)
- [6. 文件 I/O](#6-文件-io)
  - [6.1. 文件权限](#61-文件权限)
  - [6.2. umask](#62-umask)
  - [6.3. chattr](#63-chattr)
  - [6.4. 用户与用户组](#64-用户与用户组)
  - [6.5. 用户权限修改](#65-用户权限修改)
  - [6.6. 特殊权限](#66-特殊权限)
  - [6.7. 文件类型](#67-文件类型)
  - [6.8. Linux 系统目录](#68-linux-系统目录)
  - [6.9. 绝对路径与相对路径](#69-绝对路径与相对路径)
- [7. tar](#7-tar)
  - [7.1. 压缩文件类型](#71-压缩文件类型)
  - [7.2. tar文件打包](#72-tar文件打包)
  - [7.3. 打包文件或目录](#73-打包文件或目录)
  - [7.4. 解包文件或目录](#74-解包文件或目录)
  - [7.5. 解压缩文件](#75-解压缩文件)
  - [7.6. 打包压缩](#76-打包压缩)
  - [7.7. zcat zless](#77-zcat-zless)
  - [7.8. zip](#78-zip)
- [8. link](#8-link)
  - [8.1. hard links](#81-hard-links)
  - [8.2. symbolic links](#82-symbolic-links)
- [9. pipe](#9-pipe)
- [10. tee](#10-tee)
- [11. wc](#11-wc)
- [12. ps](#12-ps)
- [13. killall 与 pkill](#13-killall-与-pkill)
- [14. nohup](#14-nohup)
- [15. pstree](#15-pstree)
- [16. ltrace](#16-ltrace)
- [17. strace](#17-strace)
- [18. pstack](#18-pstack)
- [19. find](#19-find)
- [20. xargs](#20-xargs)
- [21. grep](#21-grep)
- [22. pgrep](#22-pgrep)
- [23. PID](#23-pid)
- [24. ss](#24-ss)
- [25. lsof](#25-lsof)
- [26. netcat](#26-netcat)
- [27. socat](#27-socat)
- [28. traceroute](#28-traceroute)
- [29. man](#29-man)
- [30. ntsysv](#30-ntsysv)
- [31. wget](#31-wget)
- [32. SHA](#32-sha)
- [33. md5](#33-md5)
- [34. ldconfig](#34-ldconfig)
- [35. ldd](#35-ldd)
- [36. chkconfig](#36-chkconfig)
- [37. LD_LIBRARY_PATH](#37-ld_library_path)
- [38. scp](#38-scp)
- [39. rsync](#39-rsync)
- [40. 防火墙](#40-防火墙)
  - [40.1. ubuntu 下默认的防火墙](#401-ubuntu-下默认的防火墙)
  - [40.2. CentOS 下默认的防火墙](#402-centos-下默认的防火墙)
- [41. SELinux](#41-selinux)
- [42. 共性问题](#42-共性问题)
  - [42.1. Linux 与 Windows相差 8 小时](#421-linux-与-windows相差-8-小时)
- [43. 参考](#43-参考)

<!-- /TOC -->


# 1. Linux Basic

介绍 Linux 系统的基础使用，包括计算机的硬件知识、Linux常见基础命令的用法。

----------------------------

写文章的初衷：遇见不会的知识和命令，通过网络查找，当时记住了，但过了一段时间后，再一次遇见之前的知识又要查。每次去查不仅要花费很多时间，而且找的东西良莠不齐，因此记下查的知识点，辅助自己，提高效率。

# 2. 硬件基础知识

磁盘阵列（RAID）：利用硬件技术将数个硬盘整合成为一个大硬盘的方法， 操作系统只会看到最后被整合起来的大硬盘。 由于磁盘阵列是由多个硬盘组成， 所以可以达成速度性能、 备份等任务。 


## 2.1. MBR

MBR(Master Boot Record): 主引导记录

引导启动程序记录区与分区表通常放在磁盘的第一个扇区，这个扇区通常大小为 512 bytes。
- 446字节的**MBR**安装启动引导程序，64 字节的**分区表**记录整块硬盘分区的状态。
- 由于分区表所在区块仅有64 Bytes容量， 因此最多仅能有四组记录区，每组记录区记录了该区段的启始与结束的柱面号码。 
> 引导启动程序的作用：加载内核文件。
```
1. 其实所谓的“分区”只是针对那个64 Bytes的分区表进行设置而已！
2. 硬盘默认的分区表仅能写入四组分区信息
3. 这四组分区信息我们称为主要（ Primary） 或延伸（ Extended） 分区
4. 分区的最小单位“通常”为柱面（ cylinder）
5. 当系统要写入磁盘时， 一定会参考磁盘分区表， 才能针对某个分区进行数据的处理 
```

扩展分区
- 扩展分区并不是只占一个区块，而是会分布在每个分区的最前面几个扇区来记载分区信息的！
- 扩展分区的目的是使用额外的扇区来记录分区信息， 扩展分区本身并不能被拿来格式化。 然后我们可以通过扩展分区所指向的那个区块继续作分区的记录。

MBR 主要分区(Primary)、 扩展分区(Extend)与逻辑分区(logical) 三者的区别
- 主要分区与扩展分区最多可以有四个（ 硬盘的限制）
- 扩展分区最多只能有一个（ 操作系统的限制）
- 逻辑分区是由扩展分区持续切割出来的分区；
- 能够被格式化后作为数据存取的分区是：主要分区与逻辑分区，扩展分区无法格式化；
- 逻辑分区的数量依操作系统而不同，在Linux系统中 SATA 硬盘已经可以突破63个以上的分区限制；

MBR分区的缺点
- 操作系统无法抓取到 2T 以上的磁盘容量！
- MBR 仅有一个区块，若被破坏后，经常无法或很难救援。
- MBR 内存放的启动引导程序最大为 446Bytes， 无法容纳较多的程序码。


## 2.2. GPT

GPT: GUID Partition Table，源自EFI标准的一种较新的磁盘分区表结构的标准，支持64位的寻址。

LBA(Logical Block Address): 逻辑区块位址
- LBA0(MBR兼容区块)：储存了第一阶段的启动引导程序。
- LBA1(GPT表头记录)：记录了分区表本身的位置与大小， 同时记录了备份用的 GPT 分区（在最后 34 个 LBA 区块）放置的位置，同时放置了分区表的检验机制码（ CRC32），操作系统可以根据这个检验码来判断 GPT 是否正确。 
- LBA2-33（实际记录分区的信息地方）：从 LBA2 区块开始，每个 LBA 都可以记录 4 组分区记录，所以在默认的情况下，总共可以有 4*32 = 128 组分区记录。 因为每个 LBA 有 512Bytes， 因此每组记录用到 128Bytes 的空间， 除了每组记录所需要的识别码与相关的记录之外， GPT 在每组记录中分别提供了 64bits 来记载开始/结束的扇区号码。

GPT 分区已经没有所谓的主、 扩展、 逻辑分区的概念，既然每组纪录都可以独立存在，当然每个都可以视为是主分区， 每一个分区都可以拿来格式化使用。



## 2.3. BIOS与UEFI

BIOS：是一个写入到主板上的一个软件程序（仅有16位），采用汇编语言编写的。 

Boot loader的主要任务
- 提供加载项：用户可以选择不同的启动选项，这也是多重引导的重要功能。
- 加载内核文件：直接指向可使用的程序区段，来启动操作系统。
- 转交其它启动引导程序：将启动管理功能转交给其它引导程序负责。

  > 每个分区都有自己的启动扇区(boot sector)。启动引导程序只会认识自己的系统分区内的可开机核心文件， 以及其它启动引导程序而已；

如果要安装多重开机，为什么最好先安装Windows再安装Linux呢？
> 因为 Linux在安装的时候，你可以选择将启动引导程序安装在 MBR 或其它分区的启动扇区， 而且Linux的启动引导程序可以手动设置选项，所以你可以在Linux的启动引导程序里面加入Windows启动的选项；
>Windows在安装的时候， 它的安装程序会主动的覆盖掉 MBR 以及自己所在分区的启动扇区，你没有选择的机会， 而且它没有让我们自己选择选项的功能。因此，如果先安装Linux再安装Windows的话，那么 MBR 的启动引导程序就只会有Windows的选项， 而不会有Linux的选项（ 因为原本在MBR内的Linux的启动引导程序就会被覆盖掉） 。

UEFI(Unified Extensible Firmware Interface): 统一可扩展固件接口，采用C语言编写的。其中UEFI可以直接获取GPT的分区表。

为什么在安装系统时，需要将 UEFI 的 secure boot 关闭？ 

> 因为使用secure boot会使将要启动的操作系统，必须要被UEFI验证，否则就无法启动。


## 2.4. 分区

挂载：利用一个目录当成进入点，将磁盘分区的数据放置在该目录下。其中根目录（/）必须挂载到某个分区，其它的目录可以依据用户的的需求挂载到不同的分区。

操作系统开机的流程：BIOS--->MBR--->引导启动程序--->内核文件

swap 分区：磁盘模拟内存的交换分区，当有数据被存放在物理内存里面，但这些数据又不是常被CPU使用时，这些不常被使用的数据将会被扔到硬盘的交换分区当中去，而速度较快的物理内存将被释放出来给真正需要的程序使用。交换分区不会使用目录树的挂载，所有交换分区就不需要指定挂载点。

# 3. 基础命令

## 3.1. touch 

touch fileName：文件不存在新建一个文本；文件存在时，修改文件创建的时间。

```sh
touch test.txt
```

## 3.2. rm

rm fileName(remove)：删除一个文本

```sh
rm -rf: 删除一个目录下的所有文件；以下为两个常用的参数

参数
  -i(interactive)：让系统在执行前确认。
  -r(recursive)：递归
```

## 3.3. mkdir

mkdir(make directory)：新建一个目录。

```sh
创建多层的目录加参数 -p(--parents)

一次性创建很多个文件夹
[root@CentOS7 ~]# mkdir test/bb{1..10}
[root@CentOS7 ~]# ls test/
bb1  bb10  bb2  bb3  bb4  bb5  bb6  bb7  bb8  bb9
```

## 3.4. rmdir

rmdir: 移除一个目录



## 3.5. mv 

```sh
mv 命令用来将文件或目录改名或将文件由一个目录移入另一个目录中。
mv source dest 将文件源文件或目录 (source) 移到 目标文件或目录处 (dest)，并重名为 dest。

移动文件： mv a.txt /home/Desptop/ 将当前目录下的 a.txt 文件移动到 /home/Desktop/ 路径下
移动目录： mv /usr/lib/* /home 将 /usr/lib/ 路径下的所有文件都移动到 /home/ 路径下。

参数选项：
  -v(verbose)： 显示 move 命令执行的过程。
```

## 3.6. cp 

cp 命令拷贝文件和目录。

```sh
cp src dest  将文件src拷贝到当前目录下为dest文件

注意：拷贝目录时，要加参数 -r(recursive)
```

拷贝操作会复制执行者的属性和权限。



## 3.7. ls

ls 命令列出文件夹中的内容

```sh
参数
  -a  显示所有的文件，包含隐藏的
  -l  详细的输出文件夹中内容
  -h  以人类可读的形式输出文件的大小
  --full-time  以完整的时间格式输出
  -t  根据最后修改的时间排序，最新的在第一个
  -F  在不同的文件结尾输出不同的特殊符号
  -d  显示文件夹本生信息，不输出其中的内容
  -r(reverse)  逆序排序
  -S  默认按照从小到大对文件夹大小排序
  -i  显示文件的 inode 信息
```

例子1：

```
ls -al: 查看所有隐藏的文件  
ls -l | grep "*-" | wc -l 查看当前目录下的文件夹目录个数（不包含子目录中的目录）。
ls | wc -l 统计当前目录下总共有多少行
```

例2：查看当前路径下的链接文件

```
[root@centos]# ls -lF | grep ^l
lrwxrwxrwx 1 root root       18 7月   7 2021 libcrypto.so -> libcrypto.so.1.0.0*
lrwxrwxrwx 1 root root       12 7月   7 2021 libcurl.so -> libcurl.so.4*
lrwxrwxrwx 1 root root       20 7月   7 2021 libmysqlclient.so -> libmysqlclient.so.20*
lrwxrwxrwx 1 root root       12 6月   3 2021 libpcre.so -> libpcre.so.1
```



## 3.8. stat

stat 查看文件的详细信息，比 ls 查看的更多

```sh
例子：
[root@CentOS7 ~]# stat t.txt
  File: ‘t.txt’
  Size: 149             Blocks: 8          IO Block: 4096   regular file
Device: fd00h/64768d    Inode: 34955932    Links: 1
Access: (0644/-rw-r--r--)  Uid: (    0/    root)   Gid: (    0/    root)
Context: unconfined_u:object_r:admin_home_t:s0
Access: 2021-11-20 21:20:51.966004651 +0800
Modify: 2021-11-20 21:20:44.858014426 +0800
Change: 2021-11-20 21:20:44.858014426 +0800
 Birth: -
```



## 3.9. disk

du(disk usage)：显示指定的目录或文件所占用的磁盘空间大小。

```shell
参数
  -h(human)：以人类容易看懂的方式显示
  -M(megabytes): 以1MB为单位
  -s(summarize): 仅显示总计
```

```
示例：
显示指定路径下文件的大小
[root@localhost ~]# du -sh 路径名 

统计 `~/` 路径下每个文件的大小
[root@localhost ~]# du -sh ~/*
4.0K    /root/anaconda-ks.cfg
0       /root/Desktop
0       /root/Documents
0       /root/Downloads
0       /root/Music
0       /root/Pictures
0       /root/Public
2.2M    /root/redis-6.0.16.tar.gz
0       /root/Templates
0       /root/Videos

统计当前目录下各个文件的大小
[root@localhost ~]# du -h --max-depth=1
```



## 3.10. fdisk

fdisk 操作磁盘分区表

```sh
参数：
  -l 查看硬盘的情况
```



## 3.11. df

df(disk free): 显示Linux 系统上文件系统的磁盘使用情况 

```sh
参数项：
  -h(human)：以人类容易看懂的方式显示
  -i(inodes): 列出 inode 信息，不列出已使用的 block
  -H: 很像 -h, 但是用 1000 为单位而不是用 1024
```

## 3.12. mount

mount 挂载命令

```sh
语法形式
  mount 设备名字 挂在目录  
  mount –t type dev dir

参数：
  –t type ：是需要挂载的文件系统类型，光盘文件系统类型是：iso9660；
  dev：挂载文件系统的设备名称，光盘驱动器的设备名称是/dev/cdrom; 
  dir：挂载点，即挂载到的文件目录路径

示例：
  mount -t iso9660 /dev/cdrom /media/drom

```

## 3.13. umount

umout 设备装载常用命令

```sh
示例：
  umount dir device […]
```



## 3.14. which

在 PATH 变量指定的路径中，搜索某个系统命令的位置，并且返回第一个搜索结果。

```sh
示例：`which gcc`
```



## 3.15. whereis

whereis: 查找系统中包含可以找到的所有文件

```sh
参数 -b : 只能用于搜索程序名和二进制文件

示例：whereis gcc
```

## 3.16. hostname

hostname 是 Linux 的主机名。而 Linux 的 hostname 位于 `/etc/hostname` 下，修改此路径下的文件是永久有效的。

若直接在终端使用命令 `hostname xxx` 修改，修改后仅仅是本次有效，重启后就失效了。 

# 4. 系统相关

- uname -a:  查看 Linux 版本
- `lscpu`: 查看系统 CPU 情况
- `locale -a`： 列出系统支持的所有语言环境
- `eject`: 将光盘驱动器中的光盘轻轻弹出和收回
- `nslookup 域名` 查看域名对应的IP地址

# 5. 文件查看命令

## 5.1. cat

cat(concatenate)：从第一行开始显示文件内容，将要显示的内容一次性的输出在屏幕上。

```sh
参数项
  - n 显示行号
  
示例：
  cat < hello.txt > hello2.txt   # 将 hello.txt 文件内容重定向输出到 hello.txt 文件中，相当于 cp 指令的一个副本。  
```

## 5.2. tac

tac：从最后一行开始显示文件内容。

## 5.3. nl 

nl ：查看文件时可以显示行号。

## 5.4. more

more：一页一页的显示文件内容，只能往后翻。

```
-n：可以显示行号
```

## 5.5. less

less 一页一页的显示文件内容，既可以往后翻又可以往前翻，一般用的最多。

```sh
参数：
 -n：可以显示行号
```

```
快捷键
    空格键：向下翻一页。一般使用上下箭头进行翻页。
    /字符串：向下查找字符串。
    ?字符串：向上查找字符串。
    n：重复前一个查找。
    N：反向重复前一个查找。
    g：进到这个数据的第一行。
    G：进到这个数据的最后一行。
    q：退出less程序。
```



## 5.6. od 

od(Octal Dump)：默认以二进制的方式读取文件内容。将指定文件内容以八进制、十进制、十六进制、浮点格式或 ASCII 编码字符方式显示，通常用于显示或查看文件中不能直接显示在终端的字符。

```sh
格式：od -t TYPE 文件

参数
    -A RADIX   --address-radix=RADIX   选择以何种基数表示地址偏移
    -j BYTES   --skip-bytes=BYTES      跳过指定数目的字节
    -N BYTES   --read-bytes=BYTES      输出指定字节数
    -S [BYTES] --strings[=BYTES]       输出长度不小于指定字节数的字符串，BYTES 缺省为 3
    -v         --output-duplicates     输出时不省略重复的数据
    -w [BYTES] --width[=BYTES]         设置每行显示的字节数，BYTES 缺省为 32 字节
    -t TYPE    --format=TYPE           指定输出格式，格式包括 a、c、d、f、o、u 和 x，各含义如下：
      a：利用默认的字符来输出。
      c：利用ASCII字符来输出。
      d[SIZE]：利用有符号的十进制(decimal)来输出数据。每个整数占用 SIZE bytes。
      f[SIZE]：利用浮点数(floating)来输出数据。每个浮点数占用 SIZE bytes。
      o[SIZE]：利用八进制(octal)来输出数据。每个整数占用 SIZE bytes。
      u[SIZE]：利用无符号的十进制(decimal)来输出数据。每个整数占用 SIZE bytes。
      x[SIZE]：利用十六进制(hexadecimal)来输出数据。每个整数占用 SIZE bytes。
      
      SIZE 可以为数字，也可以为大写字母。如果 TYPE 是 [doux] 中的一个，那么 SIZE 可以为
      	C  = sizeof(char)，S = sizeof(short)，I = sizeof(int)，L = sizeof(long)。
      	如果 TYPE 是 f，那么 SIZE 可以为 F = sizeof(float)，D = sizeof(double) ，L = sizeof(long double)
      
示例：
  od -t x testfile  # 以十六进制输出 testfile，默认以四字节为一组（一列）显示。
  echo abc | od -t dCc   # 查看字符的 ASCII 表
```



## 5.7. head 和 tail

head、tail：取出文件前几行或最后几行的数据。

```sh
示例：
  在屏幕上列出 /etc/man_db.conf 文件中的第11行到22行之间的内容，并且显示行号。 
  cat -n /etc/man_db.conf | head -n 20 | tail -n 10
```

## 5.8. uniq 

uniq 输出或忽略文件的重复行，常与 sort 排序命令结合使用

```sh
参数项
  -c, --count    每行前面显示重复出现的次数
  -d, --repeated 只显示重复的行
  -u, --unique   只显示出现过一次的行
```



# 6. 文件 I/O

## 6.1. 文件权限

CentOS 使用的是 `xfs` 作为默认的文件系统。

文件权限
  - 可读（Read），可以读取文件的内容。
  - 可写（Write），可以编辑、新增、或修改该文件的内容，但不具备删除该文件的权限。
  - 可执行（eXecute），Linux下，文件是否能够执行，与文件的后缀名无关，仅由是否具备 `x` 这个权限来决定。注意: X 代表这个文件具有可执行的能力，但能不能执行成功，需要由文件中的内容决定。

> 对于文件的 rwx 来说，主要都是针对“文件的内容”而言，与文件文件名的存在与否没有关系，因为文件记录的是实际的数据。文件是存放数据的所在，目录则主要记录文件名列表。因此文件名与目录有强烈的关联。


目录权限
  - r: 具有读取文件目录结构的权限
  - w: 具有改动该目录结构列表的权限。
  - x: 目录不能被执行，x 表示用户能否进入该目录并且成为工作目录。
  > 通常一个用户给其它的用户开放目录，至少要具备 `rx` 权限，其它的用户才能访问当前用户的目录。


## 6.2. umask

- umask: 指定目前用户在建立文件或目录时的默认权限值。
- 查看当前系统的umask值：`0002` ;第一个数值为特殊权限值，后面三个分别对应为 `rwx` 的值。
  > 一般文件通常用于记录数据，则用户建立的文件默认没有 `x` 可执行权限，只有 `rw` 权限，即 `-rw-rw-rw-`

  > 用户建立目录 时，默认的权限均开放，即 `drwxrwxrwx`

  > 使用 `ls -l` 查看的文件或目录权限值为：文件或目录的默认值减去 umask 的值。
  ```
  例如：umask值为 003
  文件：(-rw-rw-rw-)  - (-------wx)  = -rw-rw-r
  目录：(drwxrwxrwx)  - (-------wx)  = drwxrwx-r
  ```


## 6.3. chattr

修改文件的隐藏属性：`chattr [+-=] [ASacdistu]` 该命令一般用于对数据的安全性比较高的地方
```
选项与参数

+ ： 增加某一个特殊参数， 其他原本存在参数则不动。
- ： 移除某一个特殊参数， 其他原本存在参数则不动。
= ： 设置一定， 且仅有后面接的参数
A ： 当设置了 A 这个属性时， 若你有存取此文件（ 或目录） 时， 他的存取时间 atime 将不会被修改，可避免 I/O 较慢的机器过度的存取磁盘。（目前建议使用文件系统挂载参数处理这个项目）
S ： 一般文件是非同步写入磁盘的， 如果加上 S 这个属性时，当你进行任何文件的修改， 该更动会“同步”写入磁盘中。
a ： 当设置 a 之后， 这个文件将只能增加数据，而不能删除也不能修改数据， 只有root 才能设置这属性
c ： 这个属性设置之后， 将会自动的将此文件“压缩”， 在读取的时候将会自动解压缩，但是在储存的时候， 将会先进行压缩后再储存（ 看来对于大文件似乎蛮有用的！）
d ： 当 dump 程序被执行的时候， 设置 d 属性将可使该文件（ 或目录） 不会被 dump 备份
i ： 这个 i 可就很厉害了！ 他可以让一个文件“不能被删除、 改名、设置链接也无法写入或新增数据！”对于系统安全性有相当大的助益！ 只有 root 能设置此属性
s ： 当文件设置了 s 属性时， 如果这个文件被删除， 他将会被完全的移除出这个硬盘空间，所以如果误删了， 完全无法救回来了！
u ： 与 s 相反的， 当使用 u 来设置文件时， 如果该文件被删除了， 则数据内容其实还存在磁盘中，可以使用来救援该文件！

注意1： 属性设置常见的是 a 与 i 的设置值， 而且很多设置值必须要身为 root 才能设置
注意2： xfs 文件系统仅支持 AadiS 而已

范例： 请尝试到/tmp下面创建文件， 并加入 i 的参数， 尝试删除看看。
  [root@study ~]# cd /tmp
  [root@study tmp]# touch attrtest &lt;==创建一个空文件
  [root@study tmp]# chattr +i attrtest &lt;==给予 i 的属性
  [root@study tmp]# rm attrtest &lt;==尝试删除看看
  rm: remove regular empty file `attrtest'? y
  rm: cannot remove `attrtest': Operation not permitted
  // 看到了吗？连 root 也没有办法将这个文件删除呢！ 赶紧解除设置！

范例： 请将该文件的 i 属性取消！
[root@study tmp]# chattr -i attrtest

```

查看文件或目录的隐藏属性：`lsattr [-adR] 文件或目录`



## 6.4. 用户与用户组

说明 | 用户(owner) | 用户组(group) | 其它用户(other)
--- | --- | --- |---
权限 | 读  写  执行 | 读  写  执行| 读  写  执行
符号 | r  w  x | r  w  x | r  w  x
权值 | 4 2 1 | 4 2 1 | 4 2 1 

`ls -l`： 查看目录下文件属性的所有信息，每一栏说明如下：

  - 第1栏有10个字符，其中第1个字符描述类型，后边9个字符描述权限
  - 第2栏是硬链接数，删除文件其实是将链接数减1 ，减到0了就真正删除文件内容
  - 第3、4栏分别是文件的拥有者及所属群组。
  - 第5栏是文件大小，单位为字节（Byte）
  - 第6栏是最近访问（修改）时间
  - 第7栏是文件名，对于符号链接


## 6.5. 用户权限修改

- chown

  `chown` 命令是 change owner（改变拥有者）的缩写，修改文件的拥有者。需要要注意的是，用户必须是已经存在系统中的，也就是只能改变为在 `/etc/passwd `这个文件中有记录的用户名称才可以。

  ```
  基本语法：
    chown [-R] 用户名 文件或目录
    chown [-R] 用户名:用户组名称 文件或目录
  
  参数：
    -R : 进行递归( recursive )的持续更改，即连同子目录下的所有文件、目录都更新成为这个用户组。
         常常用在更改某一目录的情况。
  ```

  实例：

  ```
  [root@DEV ~]# ll test.txt
  -rw------- 1 root root 0 May 10 23:42 test.txt
  
  [root@DEV ~]# chown zhoush test.txt
  
  [root@DEV ~]# ll test.txt
  -rw------- 1 zhoush root 0 May 10 23:42 test.txt
  ```

  

- chgrp

  `chgrp` 命令是 change group（改变用户组）的缩写，用于修改文件的用户组。需要注意的是要改变成为的用户组名称，必须在 `/etc/group` 里存在，否则就会显示错误。

  ```
  基本语法：
    chgrp [-R] 用户组名称 dirname/filename ...
  
  参数：
    -R : 进行递归( recursive )的持续更改，即连同子目录下的所有文件、目录都更新成为这个用户组。
         常常用在更改某一目录的情况。
  ```

  ```
  [root@DEV ~]# ll test.txt
  -rw------- 1 zhoush root 0 May 10 23:42 test.txt
  
  [root@DEV ~]# chgrp zhoush test.txt
  
  [root@DEV ~]# ll test.txt
  -rw------- 1 zhoush zhoush 0 May 10 23:42 test.txt
  ```

  

- chmod

  `chmod` 命令是修改文件的模式权限，SUID、SGID、SBIT等特性
> sudo: do as su(super user)



`useradd`  增加一个新用户或者更新默认新用户信息

`usermod` 更改用户帐户属性，例如将其添加到一个已有的组中。

在 Linux 用户系统中存在两类组。第一类是**主要用户组**，第二类是**附加用户组**。所有的用户帐户及相关信息都存储在 `/etc/passwd` 文件中，`/etc/shadow` 和 `/etc/group` 文件存储了用户信息。

将一个新用户添加到主要用户组：

```
# useradd -g developers zsh
# id zsh
```

将一个新用户添加到附加用户组：

```
# useradd -G developers cnzhx
```

将已存在的用户添加到主要用户组中：

```
# usermod -g developers zsh
# id zsh
```

将一个已存在用户添加到附加用户组：

```
# usermod -G developers cnzhx
```

如果要将一个用户从某个组中删除:

```
gpasswd -d user group
```

但是这个时候需要保证 group 不是 user 的主组。

​	

参考：https://cnzhx.net/blog/linux-add-user-to-group/

## 6.6. 特殊权限

SUID(Set UID)，简写：s，数字：4
- 对于一个文件的功能：
  - SUID 权限仅对二进制程序（ binary program）有效；
  - 执行者对于该程序需要具有 x 的可执行权限；
  - 本权限仅在执行该程序的过程中有效 （ run-time）；
  - 执行者将具有该程序拥有者 （ owner） 的权限。
  > 注意：SUID仅可用在二进制程序上，不能用在shell脚本上面，因为shell脚本只是将很多的二进制可执行文件调用执行。也不能用在目录上面。
  ```
  例如：
  1. steve 对于 /usr/bin/passwd 这个程序来说是具有 x 权限的， 表示 steve 能执行passwd；
  2. passwd 的拥有者是 root 这个帐号；
  3. steve 执行 passwd 的过程中， 会“暂时”获得 root 的权限；
  4. /etc/shadow 就可以被 steve 所执行的 passwd 所修改。
  
  对于具有SUID特殊权限的文件，可以临时的将该文件的权限由steve（执行者） 变为root（拥有者）。
  ```


- SGID(Set GID)，简写：s，数字：2
  - 当文件或目录中的用大 S 表示时，代表该文件或目录本身没有可执行的 x 权限，因此 S 或 T 表示当前操作的权限为空。
  - 可以针对文件或目录操作。
  - 对文件具备的功能
    - 仅对二进制程序有用。
    - 程序执行者对于该程序来说，需具备 x 的权限；
    - 执行者在执行的过程中将会获得该程序群组的支持！
  - 对目录的功能
    - 使用者若对于此目录具有 r 与 x 的权限时， 该使用者能够进入此目录；
    - 使用者在此目录下的有效群组（ effective group） 将会变成该目录的群组；
    - 用途： 若使用者在此目录下具有 w 的权限（ 可以新建文件） ， 则使用者所创建的新文件， 该新文件的群组与此目录的群组相同。 


- SBIT(Sticky BIT)：简写：t，数字：1
  - 仅仅只针对目录有作用，对文件没有作用。
  - 对目录的功能
    - 用户对目录具有 w、x 权限时，即具有写入的权限。
    - 当用户在该目录建立文件或目录时，仅有自己与root才有删除该文件，其他人如法操作。 

> 利用数字的方式修改文件权限：将一个文件的权限改为 `-rwsr-xr-x`，命令为：`chmod 4755 filename`

> 利用符号的方式修改文件权限：SUID为 u+s，SGID为 g+s，SBIT 为 o+t

参考：https://blog.csdn.net/hudashi/article/details/7797393

## 6.7. 文件类型

类型符号 | 文件类型                      | 分类
---    |---                           | ---
\-      | 一般文件(regular)             | 纯文本文件、二进制文件、数据文件
d      | 目录文件(directory)           | null
c      | 字符设备文件(character device) | null
l      | 符号链接文件(symbolic link)    | null
p, f   | 数据传输文件(pipe, FIFO)       | null
s      | 套接文件(socket)               | null


Linux 下文件的最大长度：单一文件或目录的最大允许文件名为 255 bytes，以一个 ASCII 英文占用一个字节来说，则可达 255个 字符长度，每个汉字占用  2 个字节，则最大的文件名大约在 128 个汉字之间。


## 6.8. Linux 系统目录

FHS(Filesystem Hierarchy Standard) ：文件系统分层标准

- boot：开机启动配置文件。Linux kernel常用的文件名为：vmlinuz，如果使用的是grub2这个开机管理程序， 则还会存在/boot/grub2/这个目录。
- dev：存储外部设备文件。比要重要的文件有/dev/null, /dev/zero, /dev/tty, /dev/loop, /dev/sd等等。
-  etc：存储系统配置文件，只有root权限才可以修改
-  bin：放置系统的可执行文件
-  lib：存放开机时会用到的函数库， 以及在/bin或/sbin下面的指令会调用的函数库
-  media：放置媒体文件：软盘、光盘、DVD
-  mnt：挂载目录，这个目录的用途与/media相同，只是有了/media之后，这个目录就用来暂时挂载用了。
-  opt：第三方软件放置处
-  home：系统默认使用者的文件夹
-  root：系统管理员（root）的主文件夹。
-  sbin：放置为开机过程中所需要的，里面包括了开机、修复、还原系统所需要的指令。
-  srv：srv可以视为“service”的缩写，是一些网络服务启动之后存放的位置
-  proc：是一个虚拟文件系统，它放置的数据都在内存当中，例如系统核心、行程信息（process）、周边设备的状态及网络状态等等。这个文件本身不占任何的硬盘空间。
-  sys：这个目录其实跟/proc非常类似，也是一个虚拟的文件系统，主要也是记录内核(kernel)与系统硬件信息较相关的参数。包括目前已载入的内核模块与内核侦测到的硬件设备信息等等，这个目录同样不占硬盘容量。
-  tmp：让一般使用者或者是正在执行的程序暂时放置文件的地方，这个目录是任何人都能够存取的
-  run：早期的 FHS 规定系统开机后所产生的各项信息应该要放置到 /var/run 目录下，新版的 FHS 则规范到 /run 下面。
-  usr：是Unix Software Resource的缩写，是Unix操作系统软件资源所放置的目录，而不是使用者的数据。这个目录有点类似Windows 系统的C盘。
-  var：主要针对常态性变动（variable）的文件，包括高速缓存（cache）、登录文件（log file）以及某些软件运行所产生的文件， 包括程序文件（lock file, run file），或者例如MySQL数据库的文件等等。
-  lib64：支持 64 位的 /lib64 函数库
-  lost+found：是使用标准的ext2/ext3/ext4文件系统格式才会产生的一个目录，目的在于当文件系统发生错误时， 将一些遗失的片段放置到这个目录下。不过如果使用的是 xfs 文件系统的话，就不会存在这个目录了！
 - /usr/bin/：所有一般用户能够使用的指令都放在这里，使用链接文件的方式将 /bin 链接至此。也就是说， /usr/bin 与 /bin 是一模一样了！另外，FHS 要求在此目录下不应该有子目录。
- /usr/lib/：基本上，与 /lib 功能相同，所以 /lib 就是链接到此目录中的！
- /usr/local/：系统管理员在本机自行安装自己下载的软件，非distribution默认提供者的，建议安装到此目录， 这样会比较便于管理。
- /usr/sbin/：非系统正常运行所需要的系统指令。最常见的就是某些网络服务器软件的服务指令（daemon）！不过基本功能与 /sbin 也差不多， 因此目前 /sbin 就是链接到此目录中的。
- /usr/share/：共享数据目录，放置的数据几乎是不分硬件架构均可读取的。
- /usr/games/：与游戏相关的数据。
- /usr/include/：c/c++等程序语言的文件开始（header）与包含档（include）放置处。
- /usr/src/：Linux源代码源代码放置处。
- /usr/lib64/：与 /lib64/功能相同，因此目前 /lib64 就是链接到此目录中
- /var/cache/：应用程序本身运行过程中会产生的一些暂存盘。
- /var/lib/：程序本身执行的过程中，需要使用到的数据文件放置的目录。
- /var/lock/：设备锁存储的目录。目前此目录链接到 /run/lock 中！
- /var/log/：登录文件放置的目录！里面比较重要的文件，如/var/log/messages, /var/log/wtmp（记录登陆者的信息）等。
- /var/mail/：放置个人电子邮件信箱的目录，这个目录/var/spool/mail/链接到 /var/mail/ 目录。
- /var/run/：某些程序或者是服务启动后，会将他们的PID放置在这个目录下喔！ 与 /run 相同，这个目录链接到 /run去了！
- /var/spool/：这个目录通常放置一些伫列数据，所谓的“伫列”就是排队等待其他程序使用的数据啦！ 这些数据被使用后通常都会被删除。

```sh
一些链接文件之间的关系

/bin --> /usr/bin
/sbin --> /usr/sbin
/lib --> /usr/lib
/lib64 --> /usr/lib64
/var/lock --> /run/lock
/var/run --> /run
```


## 6.9. 绝对路径与相对路径

绝对路径：一定由跟目录(`/`)写起。例如：`/usr/share/doc`  在shell脚本中一般使用绝对路径，防止因为不同的工作环境导致一些问题的发生。

相对路径：不是由根目录(`/`)写起。例如：`../man`  相对路径只是相对于当前的工作路径。

# 7. tar

tar 命令用于的文件的打包和解压。


## 7.1. 压缩文件类型

- `gzip`: 压缩文件后缀(*.gz)
- `bzip2`: 压缩文件后缀(*.bz2)
- `xz`: 压缩文件后缀(*.xz)

<img src="./pictures/compress.png">


## 7.2. tar文件打包    

```sh
参数项：

-c(Create): 打包文件
-t(lisT): 察看打包文件的内容含有哪些文件名
-x(eXtract): 解压打包文件 
  注意：-c, -t, -x 不可同时出现在一串命令行中。
-v(Verbose): 在压缩/解压缩的过程中，将正在处理的文件名显示出来
-f(Filename):  后面要立刻接要被处理的文件名！
-C(direCtory: 目录): 将文件解压在特定的目录
-p(小写):  保存原本文件的权限与属性，不包含根目录 /。
-P(大写)：保留绝对路径，即允许备份的数据中中包含根目录。解压后的数据直接从根目录 / 开始。
-z：通过 gzip 的支持进行压缩/解压，此时文件名最好为  *.tar.gz
-j：通过 bzip2 的支持进行压缩/解压，此时文件名最好为 *.tar.bz2
-J ：通过 xz 的支持进行压缩/解压缩：此时文件名最好为 *.tar.xz
  注意：-z, -j, -J 不可以同时出现在一串命令行中
```


## 7.3. 打包文件或目录

- 将当前目录下的  anaconda-ks.cfg 文件打包成 A.tar：`tar -cvf A.tar anaconda-ks.cfg `
- 打包多个文件或目录，中间需要用空格分开：`tar -cvf B.tar anaconda-ks.cfg /tmp/`


## 7.4. 解包文件或目录

-  格式：`tar -xvf 解压的文件  -C 文件解压后的路径`
-  注意：若后面不跟 `-C 文件解压后的路径` ，则会默认解包到当前路径下
-  `tar -xvf test.tar -C /tmp` ：将test.tar打包的文件解包到 /tmp 路径下


## 7.5. 解压缩文件

- 解多个文件

  `tar -zxvf etc.tar.gz -C ~/Exercise_Linux/tmp`       将etc.tar.gz文件解压到~/Exercise_Linux/tmp目录下，不加 `-C ~/Exercise_Linux/tmp ` 则只是解压到当前目录下。

  注意：指定解压的目录必须要首先存在，否则会出错，tar命令不会自动创建不存在的文件夹。


- 解压单一文件

  `tar -zxvf etc.tar.gz etc/gdb`   将 etc.tar.gz压缩包中gdb文件夹解压 到当前目录下。


## 7.6. 打包压缩

- 打包压缩所有文件
  
  `tar -zcvf 自己创建的文件名(xxx.tar.gz)  要打包压缩的路径(/etc/)`: 这样压缩的文件连要压缩文件的目录也一起给压缩了。
  
- 打包压缩所有文件不包含打包文件的路径
  
  `tar -zcvf tmp4.tar.gz -C etc/ .`    将etc下所有文件打包为tmp4.tar.gz 不包含etc包的路径。
  
- 打包并压缩一个目录，但不含该目录下的某些文件
  
	`tar -zcvf bb.tar.gz --exclude=etc/apt etc`   将etc目录下除去apt文件的所有文件打包压缩为 bb.tar.gz，打包压缩时包含etc的路径。
  
    1.  一定要注意排除目录的最后不要带 `/`，否则 `exclude` 目录将不起作用
    2.  压缩目录和排除目录都需要采用同样的格式，如都采用绝对路径或者相对路径


## 7.7. zcat zless 

zcat、zless 命令直接查看压缩文件中的内容。

```sh
$ zcat test.gz
hello word

$ zless test.gz
hello word
```


## 7.8. zip

zip 是压缩指令，unzip 是解压指令。zip 指令既可以压缩文件，也可以压缩目录。压缩会自动保留源文件，解压会自动保留压缩文件。

```
// 将demo.txt文件和目录mydir压缩成压缩文件yasuo.zip，选项-r表示递归
zip -r yasuo.zip demo.txt mydir  

// 压缩当前目录下的子目录mydir
zip -r  mydir.zip  mydir         

// 解压yasuo.zip文件到当前目录
unzip   yasuo.zip                

// 把压缩文件解压到指定的mydir目录
unzip -d /mydir yasuo.zip        

// 检查压缩文件是否损坏
unzip -t  yasuo.zip              

// 显示demo.zip压缩包中有哪些文件，不进行解压
unzip  -l  demo.zip              

// 解压时不覆盖已存在的文件
unzip  -n  demo.zip              
```

注意：直接使用unzip指令（不带选项）解压文件时，如果解压文件中包含有文件与当前目录下的某个文件重名，那么会询问是否要覆盖这个文件。

# 8. link

Linux 下用 `ln` 来执行链接。`ln` 后面不加 `-s` 参数表示进行硬链接操作，加参数表示软连接操作。


## 8.1. hard links

硬链接: 指向磁盘中文件的节点(inode),只有文件才能创建硬链接，目录不能创建。

硬链接会创建独立的虚拟文件，其中包含了原始文件的信息及位置。但是它们从根本上而言是同一个文件。引用硬链接文件等同于引用了源文件。要创建硬链接，原始文件也必须事先存在，只不过这次使用ln命令时不再需要加入额外的参数了。

```
// 建立硬链接
ln 原文件 新文件 
```



## 8.2. symbolic links

符号链接就是一个实实在在的文件，它指向存放在虚拟目录结构中某个地方的另一个文件。这两个通过符号链接在一起的文件，彼此的内容并不相同。

要为一个文件创建符号链接，原始文件必须事先存在。然后可以使用 `ln` 命令以及 `-s`选项来创建符号链接。

```
// 建立软连接
ln -s source  destination 
```

# 9. pipe

pipe 中文翻译过来是管道的意思，用 `|` 表示。

定义：将一个命令的输出传送给另一个命令，作为另一个命令的输入，常用 `|` 表示。管道常与grep命令组合使用：`grep 命令1|命令2|命令3|·····|命令n`

& 和 &&  | 和 || 四者区别
- `& `: 表示将当前任务放在后台执行，如要在后台运行 redis-server，则有  redis-server &
- `&&`: 表示前一条命令执行成功时，才执行后一条命令，如 echo 'hello‘ && echo 'world'    
- `| `: 表示管道，上一条命令的输出，作为下一条命令参数，如 echo 'hello' | wc -l
- `||`: 表示上一条命令执行失败后，才执行下一条命令，如 cat nofile || echo "failed"

# 10. tee

man手册英文原意：`tee - read from standard input and write to standard output and files`

- 功能：从标准输入读数据，写到标准输出和文件中。
- 用法：`echo hello | tee file` 将 hello 字符写到file文件中并显示在标准输出上。

# 11. wc

作用：print newline, word, and byte counts for each file. (用来统计一个文件或者指定的多个文件中的行数，单词数和字符数)

```sh
格式
  wc [OPTION]... [FILE]...
  
选项参数
  -c --bytes 打印字节数
  -l --lines 打印行数
  -w --words 打印单词数
  -m --chars 打印字符数
  -L --max-line-length 打印最长行的长度
```


# 12. ps

ps 是进程状态（process status）的缩写，用于查看 Linux 系统中进程的状态情况。

```
~$ ps aux
USER        PID %CPU %MEM    VSZ   RSS TTY      STAT START   TIME COMMAND
root          1  0.0  0.3 128324  6960 ?        Ss   13:58   0:02 /usr/lib/systemd/systemd --switched-root --system --deserialize 22
root          2  0.0  0.0      0     0 ?        S    13:58   0:00 [kthreadd]
root          4  0.0  0.0      0     0 ?        S<   13:58   0:00 [kworker/0:0H]
root          5  0.0  0.0      0     0 ?        S    13:58   0:00 [kworker/u256:0]
root          6  0.0  0.0      0     0 ?        S    13:58   0:00 [ksoftirqd/0]
root          7  0.0  0.0      0     0 ?        S    13:58   0:01 [migration/0]
root          8  0.0  0.0      0     0 ?        S    13:58   0:00 [rcu_bh]
root          9  0.0  0.0      0     0 ?        S    13:58   0:25 [rcu_sched]

```

显示格式参数

```sh
USER：用户名
%CPU：该进程用掉的CPU百分比
%MEM：进程占用内存的百分比
VSZ：该进程使用的虚拟內存量（KB）
RSS：该进程占用的固定內存量（KB）（驻留中页的数量）
TTY：表示该进程在那个终端上运行，若与终端无关，则显示? 若为pts/n，则表示由网络连接进入主机的进程，tty1-tty6 表示是本机上面的登录进程。
STAT：进程当前的状态
    D(uninterruptible sleep) 不可中断的休眠状态（通常 IO 相关的进程）
    I(idle)                  空闲的内核线程      
    R(running or runnable)   在运行队列中进程状态未：正在运行态或可运行状态
    S(大写：sleep)            可中断睡眠状态（等待一个事件完成）
    T(stopped)               停止状态（stopped by job control signal）
    t                        调试追踪过程中被停止的状态（stopped by debugger during the tracing）
    W（paging）               进入内存交换 （从内核2.6开始无效）
    X(dead)                  死掉的进程 （基本很少看见）
    Z(zombie)                僵尸状态，进程已被终止，但无法被删除
    <                        高优先级进程（not nice to other users）
    N                        低优先级的进程（nice to other users）
    L                        页被锁进内存
    s                        该进程含有子进程
    l                        多线程，克隆线程（使用 CLONE_THREAD, 类似 NPTL pthreads）
    +                        位于前台的进程组（foreground process group）
    
START：该进程被触发启动的时间
TIME：该进程实际使用CPU运行的时间
CMD(command)：执行此进程触发的命令是什么
UID：用户ID、但输出的是用户名
PID：进程的ID
PPID：父进程ID
C：CPU使用率，单位为百分比
STIME：进程启动到现在的时间
PRI(priority)：进程被CPU执行的优先级，数值越小，代表该进程被CPU执行的越快。这个值由内核动态调整，用户无法直接调整PRI的值。
NI(nice)：调整进程的优先级。
    nice值的可调整的范围在 -20~19 之间。
    root用户可以随意调整自己或其它用户进程的 nice值，且范围范围在 -20~19 之间。
    一般用户只能调整自己进程的nice值，范围仅为 0-19，避免一般用户去抢占系统的资源。
    PRI与NI之间的关系：PRI(new) = PRI(old) + nice
    nice值有正负，当nice值为负数时，那么该进程会降低PRI的值，会变得较优先处理。
    如何调整nice值？
      1. 进程刚开始时就给指定一个特定的nice值。nice -n -5 vim & 启动vim时，给定一个nice值，并将vim放在后台执行。
      2. 调整已存在的进程的nice值，需要用 renice 命令：renice 4 2366 将PID=2366 进程的nice值调整为4

ADDR：是内核函数，指出该进程在内存中的哪个部分；如果是个running的进程，一般用 - 表示
SZ：此进程用掉多少内存
WCHAN：目前进程是否在运行，如果为 -，则表示正在运行。

```


常见参数命令组合
- `ps –ef|grep 程序名称`：查看一个程序是否运行 
- `ps -Lf 端口号|wc -l `：查看线程个数  
- `ps -l`：查看当前用户的 bash 进程
- `ps aux`：查看系统运行的所有进程，默认按照 PID 的顺序排序。
- `ps axjf`：查看系统运行的所有进程，并带有 PPID 项


```sh
# 查看某个进程已运行的时间
john@ubuntu:~$ ps -p 3578 -o lstart,etime
                STARTED     ELAPSED
Thu Jun 10 08:33:04 2021       33:48
```


# 13. killall 与 pkill

根据进程的名称去杀死进程，而不需要知道进程的 ID 号。

```sh
killall bash

pkill bash
```

# 14. nohup

```sh
nohup 命令让程序在后台执行，一般常与 & 符号结合使用。

示例：
  nohup ping www.baidu.com    让执行 ping 命令的进程在后台运行。
```

# 15. pstree 

pstree 命令是用于查看正在运行的进程之间的关系，用树形图显示，即哪个进程是父进程，哪个是子进程，可以清楚的看出来是谁创建了谁，同时还可以查看一个进程下有多少个子线程。

注：Linux 系统中内核调用的第一个进程为 `systemd`，该进程的 PID 为 `1`。

```
几个重要的参数：
-A 各进程树之间的连接以ASCII码字符来连接
-U 各进程树之间的连接以utf8字符来连接，某些终端可能会有错误
-p 同时列出每个进程的PID
-u 同时列出每个进程的所属账号名称
```

```
$ pstree
init─┬─NetworkManager
     ├─abrtd
     ├─acpid
     ├─atd
     ├─automount───4*[{automount}]
     ├─certmonger
     ├─crond
     ├─cupsd
     ├─dbus-daemon
     ├─hald───hald-runner─┬─hald-addon-acpi
     │                    └─hald-addon-inpu
     ├─irqbalance
     ├─master─┬─bounce
```

```
// 一个进程下有多个子线程，花括号{} 中的内容表示线程，圆括号() 中的内容表示线程ID或进程ID
pstree -p 15821
a.out(15821)─┬─{a.out}(15835)
             ├─{a.out}(15836)
             ├─{a.out}(15837)
             ├─{a.out}(15838)
             ├─{a.out}(15839)
             ├─{a.out}(15840)
             ├─{a.out}(15841)
             ├─{a.out}(15843)
             ├─{a.out}(15844)
```

# 16. ltrace

跟踪进程调用库函数的信息，显示调用哪些库函数。

```
语法
  ltrace [option ...] [command [arg ...]]

选项参数
-a 对齐具体某个列的返回值。
-c 计算时间和调用，并在程序退出时打印摘要。
-C 解码低级别名称（内核级）为用户级名称。
-d 打印调试信息。
-e 改变跟踪的事件。
-f 跟踪子进程。
-h 打印帮助信息。
-i 打印指令指针，当库调用时。
-l 只打印某个库中的调用。
-L 不打印库调用。
-n, --indent=NR 对每个调用级别嵌套以NR个空格进行缩进输出。
-o, --output=file 把输出定向到文件。
-p PID 附着在值为PID的进程号上进行ltrace。
-r 打印相对时间戳。
-s STRLEN 设置打印的字符串最大长度。
-S 显示系统调用。
-t, -tt, -ttt 打印绝对时间戳。
-T 输出每个调用过程的时间开销。
-u USERNAME 使用某个用户id或组ID来运行命令。
-V, --version 打印版本信息，然后退出。

示例
  ltrace -p pid
```

示例1：执行程序不带任何参数

```
$ ltrace ./a.out
(0, 0, 176780, -1, 0x7f080f441990)                                                            = 0x304c2221e0
__libc_start_main(0x400898, 1, 0x7fff96a76768, 0x400920, 0x400990 <unfinished ...>
_ZNSt8ios_base4InitC1Ev(0x600ee0, 65535, 0x7fff96a76778, 64, 0x304c7bbea0)                    = 0
__cxa_atexit(0x400704, 0x600ee0, 0x600d98, 6, 0x305a3105e0)                                   = 0
_ZStlsISt11char_traitsIcEERSt13basic_ostreamIcT_ES5_PKc(0x600dc0, 0x4009ec, 0x7fff96a76778, 96, 0x304c7bbea0) = 0x600dc0
_ZNSolsEPFRSoS_E(0x600dc0, 0x400734, 0x305a2f75d8, 1024, 0x304c7bca30 <unfinished ...>
_ZSt4endlIcSt11char_traitsIcEERSt13basic_ostreamIT_T0_ES6_(0x600dc0, 0x400734, 0x305a2f75d8, 1024, 0x304c7bca30hello
 <unfinished ...>
<... _ZNSolsEPFRSoS_E resumed> )                                                              = 0x600dc0
_ZNSt8ios_base4InitD1Ev(0x600ee0, 0, 160, 0x304c7bbf30, 4)                                    = 0x305a30f240
+++ exited (status 0) +++
```

示例2：输出调用时间开销

```
$ ltrace -T ./a.out
(0, 0, 106015, -1, 0x7f3655ca9990)                                                            = 0x304c2221e0 <0.000129>
__libc_start_main(0x400898, 1, 0x7fff2085dd98, 0x400920, 0x400990 <unfinished ...>
_ZNSt8ios_base4InitC1Ev(0x600ee0, 65535, 0x7fff2085dda8, 64, 0x304c7bbea0)                    = 0 <0.000215>
__cxa_atexit(0x400704, 0x600ee0, 0x600d98, 6, 0x305a3105e0)                                   = 0 <0.000110>
_ZStlsISt11char_traitsIcEERSt13basic_ostreamIcT_ES5_PKc(0x600dc0, 0x4009ec, 0x7fff2085dda8, 96, 0x304c7bbea0) = 0x600dc0 <0.000221>
_ZNSolsEPFRSoS_E(0x600dc0, 0x400734, 0x305a2f75d8, 1024, 0x304c7bca30 <unfinished ...>
_ZSt4endlIcSt11char_traitsIcEERSt13basic_ostreamIT_T0_ES6_(0x600dc0, 0x400734, 0x305a2f75d8, 1024, 0x304c7bca30hello
 <unfinished ...>
<... _ZNSolsEPFRSoS_E resumed> )                                                              = 0x600dc0 <0.000302>
_ZNSt8ios_base4InitD1Ev(0x600ee0, 0, 160, 0x304c7bbf30, 4)                                    = 0x305a30f240 <0.000141>
+++ exited (status 0) +++
```

示例3：显示系统调用

```
$ ltrace -S ./a.out
SYS_brk(NULL)                                                                                 = 0x23f8000
SYS_mmap(0, 4096, 3, 34, 0xffffffff)                                                          = 0x7f16eaa25000
SYS_access(0x304c01dac0, 4, 6, 4, 0x2f7362694c2f564f)                                         = -2
SYS_open("tls/x86_64/libstdc++.so.6", 524288, 011410421010)                                   = -2
SYS_open("tls/libstdc++.so.6", 524288, 011410421010)                                          = -2
SYS_open("x86_64/libstdc++.so.6", 524288, 011410421010)                                       = -2
SYS_open("libstdc++.so.6", 524288, 011410421010)                                              = -2   
...........
SYS_munmap(0x7f16eaa16000, 59833)                                                             = 0
__libc_start_main(0x400898, 1, 0x7fff976dc088, 0x400920, 0x400990 <unfinished ...>
_ZNSt8ios_base4InitC1Ev(0x600ee0, 65535, 0x7fff976dc098, 64, 0x304c7bbea0)                    = 0
__cxa_atexit(0x400704, 0x600ee0, 0x600d98, 6, 0x305a3105e0)                                   = 0
_ZStlsISt11char_traitsIcEERSt13basic_ostreamIcT_ES5_PKc(0x600dc0, 0x4009ec, 0x7fff976dc098, 96, 0x304c7bbea0 <unfinished ...>
SYS_fstat(1, 0x7fff976dbdb0, 0x7fff976dbdb0, 0x7fff976dbcd0, 0x304c7bca30)                    = 0
SYS_mmap(0, 4096, 3, 34, 0xffffffff)                                                          = 0x7f16eaa24000
<... _ZStlsISt11char_traitsIcEERSt13basic_ostreamIcT_ES5_PKc resumed> )                       = 0x600dc0
_ZNSolsEPFRSoS_E(0x600dc0, 0x400734, 0x305a2f75d8, 1024, 0x304c7bca30 <unfinished ...>
_ZSt4endlIcSt11char_traitsIcEERSt13basic_ostreamIT_T0_ES6_(0x600dc0, 0x400734, 0x305a2f75d8, 1024, 0x304c7bca30 <unfinished ...>
SYS_write(1, "hello\n", 6hello
)                                                                    = 6
<... _ZNSolsEPFRSoS_E resumed> )                                                              = 0x600dc0
_ZNSt8ios_base4InitD1Ev(0x600ee0, 0, 160, 0x304c7bbf30, 4)                                    = 0x305a30f240
SYS_exit_group(0 <no return ...>
+++ exited (status 0) +++
```

参考：[ltrace命令详解 ](https://www.cnblogs.com/machangwei-8/p/10388938.html)

# 17. strace

**为什么要用 strace?**

看到一个进程调用了哪些API以及其调用顺序，例如我们要参考某个程序的实现，但我们又无法获得该程序的源代码时，使用系统调用跟踪命令不失为一个好办法。另外，在一些无法调试的环境上检查问题时，我们也可以用该命令来查看程序是否按预期执行。strace和dtruss都是同一类型的命令，strace是linux系统上的，而dtruss是mac系统上的。

**strace 作用**

strace 跟踪一个进程的系统调用或信号产生的信息。（strace  - trace system calls and signals）

```sh
参数项

-c 统计每一系统调用的所执行的时间，次数和出错的次数等。 
-d 输出strace关于标准错误的调试信息。 
-f 跟踪由fork调用所产生的子进程。 
-ff 如果提供-o filename，则所有进程的跟踪结果输出到相应的filename 
-F 尝试跟踪vfork调用。在-f时，vfork不被跟踪。 
-h 输出简要的帮助信息。 
-i 输出系统调用的入口指针。 
-q 禁止输出关于脱离的消息。 
-r 打印出相对时间关于，每一个系统调用。 
-t 在输出中的每一行前加上时间信息。 
-tt 在输出中的每一行前加上时间信息，微秒级。 
-ttt 微秒级输出，以秒了表示时间。 
-T 显示每一调用所耗的时间。 
-v 输出所有的系统调用。一些调用关于环境变量，状态，输入输出等调用由于使用频繁，默认不输出。 
-V 输出strace的版本信息。 
-x 以十六进制形式输出非标准字符串 
-xx 所有字符串以十六进制形式输出。 
-a column 设置返回值的输出位置。默认 为40。 

-e expr  指定一个表达式，用来控制如何跟踪。格式如下: 
	[qualifier=][!]value1[，value2]。。。 
	qualifier只能是 trace，abbrev，verbose，raw，signal，read，write其中之一。
	value是用来限定的符号或数字。默认的 qualifier是 trace。感叹号是否定符号。
	例如: -eopen等价于 -e trace=open，表示只跟踪open调用。
	而-etrace!=open表示跟踪除了open以外的其他调用。有两个特殊的符号 all 和 none。 
 
-e trace=set 只跟踪指定的系统 调用。
             例如:-e trace=open，close，rean，write表示只跟踪这四个系统调用。默认的为set=all。
			 
-e trace=file 只跟踪有关文件操作的系统调用。 

-e trace=process 只跟踪有关进程控制的系统调用。 

-e trace=network 跟踪与网络有关的所有系统调用。
 
-e strace=signal 跟踪所有与系统信号有关的 系统调用 

-e trace=ipc 跟踪所有与进程通讯有关的系统调用 

-e abbrev=set 设定 strace输出的系统调用的结果集

-e raw=set 将指定的系统调用的参数以十六进制显示。 

-e signal=set 指定跟踪的系统信号。默认为all。
              如 signal=!SIGIO(或者signal=!io)，表示不跟踪SIGIO信号。 

-e read=set  输出从指定文件中读出 的数据。例如: 

-e read=3，5 

-e write=set 输出写入到指定文件中的数据。 

-o filename 将strace的输出写入文件filename 

-p pid 跟踪指定的进程pid。 

-s strsize 指定输出的字符串的最大长度，默认为32。 

-u username 以username 的UID和GID执行被跟踪的命令
```




# 18. pstack
pstack 打印正在运行的进程的堆栈信息。


# 19. find
find 命令：在指定的目录中共去查找文件。 

```sh
格式
  find [-H] [-L] [-P] [-D debugopts] [-Olevel] [path...] [expression]

选项参数
-name：按照名字匹配搜索，搜索的内容要用引号括起来，单引号或双引号都可以，从而避免它受 shell 终端扩展的影响。
	例子：按照文件查找 /usr/src 路径下后缀名为 .txt 所有文件
	find /usr/src -name "*.txt"
  
-type：按照文件类型查找。
       类型包括：f(file), d(directory), l(link), c(char), d(device), s(socket), b(block)
 	例子：按照文件类型查找 /usr/src 路径下后缀名为 .txt 所有文件
 	find /usr/src -type "*.txt"
  
-size: 按照文件大小搜索，默认单位为512byte，一个扇区的大小
	例子：find /usr/src -size +10M -size -20M 查找大于10M小于20M的文件
	例子：find /usr/src -size +10k -size -20k
  
-maxdepth: 
	例子：统计 /usr 目录下深度为2的所有目录文件 
	find /usr -maxdepth 2 -type d | wc -l  
	
	例子：查找 /usr 路径下深度为 2 除开类型为目录的所有文件
	find /usr -maxdepth 2 ! -type d 
	
-exec
	例子： 列出当前目录下所有的 .sh 文件，并执行ls -l 命令
	find ./ -name "*.sh" -exec ls -l {} \;  

-print: 将文件或目录名称列出到标准输出。格式为每列一个名称，每个名称前皆有 ./ 字符串；

-print0: 就将文件或目录名称列出到标准输出。格式为全部的名称皆在同一行；

-atime(access time): 访问时间， +7 超过七天被访问的文件；-7 七天以内访问过的文件; 7 恰好在七天前被访问的文件（那个时间点）
	例子；查找当前路径下恰好在七天前被访问的文件
	find . atime -7 
  
-amin: 访问时间（按照分钟）

-mtime: 上次修改的时间（按照天数）
	查找当前路径下 7 天以前的 ErrorLog 文件
	find . -name "ErrorLog*.log" -mtime +7
	
	删除当前路径下超过 10 天被修改的文件
	find . -name "DataDeal*.log" -mtime +10 | xargs rm -rf
	
	另外一种写法
	find . -name "IOV-GBDataDeal*.log" -mtime +10 -exec rm -rf {} \;

-mmin(modified minute): 修改时间（按照分钟）

-ctime(change time): 最近文件的状态被改变的时间

-cmin(change minute): 最近文件的状态被改变的时间（按照分钟）

-prune: 忽略指定的文件查找
   find . -path "./test" -prune -o -name "*.txt" -print  忽略 test 文件夹去查找当前路径下以 txt 结尾的所有文件
   
-ok 执行的命令：输出的结果确定是否要执行指定的命令
	find .  -path "./" -prune -o -name "*.gz" -ok ls -l {} \;

-iname
	例子：查找当前路径下所有 .log 文件中包含的 open files 字段
  	find . -iname '*.log' | xargs grep 'open files'
  		
    从根目录开始查找所有扩展名为 .log 的文本文件，并找出包含 "flower" 的行：
	$ find / -type f -name "*.log" | xargs grep "flower"
	
	从当前目录开始查找所有扩展名为 .ini 的文本文件，并找出包含 "dog" 的行：	
	find . -name "*.ini" | xargs grep "dog"
```

# 20. xargs

xargs 又称管道命令。是给命令传递参数的一个过滤器，也是组合多个命令的一个工具，它把一个数据流分割成一些足够小的快，方便过滤器和命令进行处理。

```sh
参数项
  -d 指定一个特定的分隔符显示，默认分隔符为空格
  -i 使用 {} 替代传递的参数
  -n 限制单个命令行的参数个数
  -t 显示详情
  -p 交互模式
  -0 --null，使用 null 分割，而不是空白，禁用引号和反斜杠处理

例子：
  多行输入变单行
    $ cat a.txt
    123
    456
    789
    
    $ xargs < a.txt
    123 456 789
    
  -n 参数的使用
    $ cat a.txt
    123 111 222 333
    456 444 555 666
    789 777 888 999
    
    $ xargs -n 2 < a.txt
    123 111
    222 333
    456 444
    555 666
    789 777
    888 999
    
  -d 参数使用
    $  echo "helo,AI,DB,CJ,KK"
    helo,AI,DB,CJ,KK
    
    $  echo "helo,AI,DB,CJ,KK" | xargs -d ","
    helo AI DB CJ KK
    
    $  echo "helo,AI,DB,CJ,KK" | xargs -d "," -n 2
    helo AI
    DB CJ
    KK
    
 -i参数使用
 	将当前目录下深度为 1 的所有 .txt 文件移动到当前已存在的 temp 目录下；xargs -i 作为参数传递
    $ find . -maxdepth 1 -name "*.txt" | xargs -i mv {} ./temp
    $ ls temp/
    a.txt  b.txt
    
  -I参数使用
  	将当前目录下所有为 .txt 的文件移出到上一级已存在的 txt 目录下
    [root@CentOS7 temp]# find . -name "*.txt" | xargs -I aa mv aa ../txt
    $ ls txt/
    a.txt  b.txt
```

# 21. grep

`grep(global search regular expression and print out the line)` 全面搜索正则表达式和打印输出行 

三种形式的 grep 命令

- gerp 是标准格式
- egrep 是扩展grep命令，其实和grep -E等价，支持基本和扩展的正则表达式。
- fgrep 是快速grep命令，其实和grep -F等价，不支持正则表达式，按照字符串表面意思进行匹配。

```sh
用法
  grep [OPTIONS] PATTERN [FILE...]
  grep [OPTIONS] [-e PATTERN | -f FILE] [FILE...]

OPTIONS: 
  通用程序信息（Generic Program Information）
    --help 输出帮助信息后退出
    
    -V, --version 
           输出 grep 版本号后退出
    
  模式语法（Pattern Syntax）
    -E, --extended-regexp
           匹配扩展正则表达式
           
    -F, --fixed-strings
          匹配固定字符串而非正则表达式
          
 	-G, --basic-regexp
          匹配基本的正表达式，这个是默认选项
          grep '[^0-6]'  helo.txt
          
    -P, --perl-regexp
          匹配 perl 语法的正则表达式
          
  匹配控制（Matching Control）
  	-i, --ignore-case
  	      查找时忽略大小写：grep -i "book"
          
	--no-ignore-case
	     匹配时不忽略大小写
	     
    -v, --invert-match
         选择不包含所匹配文本的行
 	-w, --word-regexp
          匹配整个单词
          
	-x, --line-regexp
	      匹配整行
	      
 通用输出控制
   -c, --count
         只输出匹配行的数量
         
   -l, --files-with-matches
         只列出符合匹配的文件名，不列出具体的匹配行  
         
   -q, --quiet, --silent 
         退出：不写任何的东西到标准输入中。如果找到任何匹配项，立即退出，状态为零，即使检测到错误也是如此。
         
  -s, --no-message 
        不显示不存在、没有匹配文本的错误信息  
        
   -o, --only-matching     
         与-b结合使用，打印匹配的词据文件头部的偏移量，以字节为单位
         
输出行前缀控制（Output Line Prefix Control）
  -b, --byte-offset
        打在每行输出之前打印输入文件中从 0 开始的字节偏移量。
        
  -h, --no-filename 
      查询多文件时不显示文件名      
      
  -n, --line-number
        列出所有的匹配行，显示行号

文件和路径选择（File and Directory Selection）
	-a, --text
	      将处理的二进制文件当作文本文件，等同于 --binary-files=text 操作
          例子： 搜索压缩文件中的 open files 关键字
          $ zcat Server_log/20220118.tar.gz  | grep -a "open files"

	-r, --recursive 
    递归搜索
      例子： 搜索 /usr/src/ 路径下包含 task_struct { 的字符，并显示字符所在的行号
      grep -r  "task_struct {" /usr/src/  -n 
```



查询生产环境下以压缩归档的日志，在不用解压文件的前提下直接进行查询日志，其中压缩的文件格式为 `.gz`

```
[root@John]# zgrep -ia "ReportDBTable" ./Server_log/20220411.tar.gz
[20220411_204121_750][I]received update config[t_config.ReportDBTable]: old(), new(1)[TID:4661]
[20220411_204214_321][I]Get ReportDBTable value: 1[TID:3143]
```



# 22. pgrep

根据进程的名称查找并返回进程的 ID 号到标准输出。与 pidof 功能一样。

```sh
pgrep -l program_name  只显示某个进程的PID
pidof program_name  找出某个正在执行的进程的PID

pgrep bash
3528

pidof bash 
3528
```

# 23. PID

- `ps aux | grep xxx(程序名称)`  显示某个进程的全部信息，包括PID
- `ps ajx` 显示进程组ID
- `ulimit -a` 查看资源的上限大小 

# 24. ss 

ss 是用于调查套接字的另一个实用程序。

```sh
ss 检查端口： 

示例：
  ss -tunlp
```



# 25. lsof 

lsof(list open files) 列出整个 Linux 系统打开的所有文件描述符。

```
参数
  -p：指定进程ID（ PID）。
  -d：允许指定要显示的文件描述符编号。
  
   参看文件描述符为 1 的进程
   [root@KF-CFT-AP2 ~]# lsof -d 1
   COMMAND     PID      USER   FD   TYPE             DEVICE SIZE/OFF     NODE NAME
   init          1      root    1u   CHR                1,3      0t0     3842 /dev/null
   udevd       557      root    1u   CHR                1,3      0t0     3842 /dev/null

  -i:端口号  查看指定端口占用情况
 	查看 47462 端口的占用情况
  [root@KF]# lsof -i:47462
  COMMAND     PID USER   FD   TYPE   DEVICE SIZE/OFF NODE NAME
  L2BU-Boot 27283 root   19u  IPv4 82427778      0t0  TCP 172.26.153.222:47462->172.26.153.227:12243 (ESTABLISHED)
 
  
示例：
  lsof -nP -iTCP -sTCP:LISTEN 获取所有侦听 TCP 端口的列表 
```

lsof 命令输出各列信息的意义如下：

- `COMMAND`：进程的名称
- `PID`：进程标识符
- `USER`：进程所有者
- `FD`：文件描述符类型
- `TYPE`：文件类型
- `DEVICE`：指定磁盘的名称
- `SIZE/OFF`：文件的大小
- `NODE`：文件在磁盘上的标识，索引节点
- `NAME`：打开文件的确切名称

> Tips：查看 xxx 端口的占用情况，有两种方式。第一采用 `lsof -i:xxx` 查看；第二：采用 `netstat -tunlp | grep xxx` 来查看。


# 26. netcat

netcat（通常缩写为nc）是一种计算机联网实用程序，用于使用TCP或UDP读写网络连接。 该命令被设计为可靠的后端，可以直接使用或由其他程序和脚本轻松驱动。 同时，它是功能丰富的网络调试和调查工具，因为它可以产生用户可能需要的几乎任何类型的连接，并具有许多内置功能。netcat被称为网络工具中的瑞士军刀，体积小巧，但功能强大。


# 27. socat

Socat 是 Linux 下的一个多功能的网络工具，名字来由是 「Socket CAT」。其功能与有瑞士军刀之称的 Netcat 类似，可以看做是 Netcat 的加强版。socat的官方网站：http://www.dest-unreach.org/socat/ 

Socat 的主要特点就是在两个数据流之间建立通道，且支持众多协议和链接方式。如 IP、TCP、 UDP、IPv6、PIPE、EXEC、System、Open、Proxy、Openssl、Socket等。


# 28. traceroute

追踪从出发地（源主机）到目的地（目标主机）之间经过了哪些路由器，以及到达各个路由器之间的消耗的时间。默认发送的数据包大小是40字节。


# 29. man

man是 POSIX(Portable Operating System Interface) 规定的帮助手册程序。

```sh
语法格式：man -n 命令参数

其中 n 为数字，不同的数字表示如下：
    1：普通应用程序或shell命令（Executable programs or shell commands）
    2：系统调用（System calls (functions provided by the kernel)）
    3：库函数（Library calls (functions within program libraries)）
    4：设备文件（Special files (usually found in /dev)）
    5：文件格式、或相关协议（File formats and conventions, e.g. /etc/passwd）
    6：游戏设备（Games）
    7：其它设备（Miscellaneous (including macro packages and conventions), e.g. man(7), groff(7)）
    8：root管理命令（System administration commands (usually only for root)）
    9：非标准的内核程序（Kernel routines [Non standard]）
    
示例：
  查找 socket 文档： man 2 socket
```

<img src="./pictures/man代号.png">


  ```sh
  man -k keyword    # 按照关键字搜索与之匹配的相似命令。
  
  john@ubuntu:~$ man -k what
  git-blame (1)        - Show what revision and author last modified each line of a file
  git-receive-pack (1) - Receive what is pushed into the repository
  git-whatchanged (1)  - Show logs with difference each commit introduces
  imgtoppm (1)         - convert an Img-whatnot file into a portable pixmap
  lwp-dump (1p)        - See what headers and content is returned for a URL
  w (1)                - Show who is logged on and what they are doing.
  w.procps (1)         - Show who is logged on and what they are doing.
  whatis (1)           - display one-line manual page descriptions
  ```

man 手册中的一些关键字

| 英文描述      | 中文描述                     |
| ------------- | ---------------------------- |
| NAME          | 命令名                       |
| SYNOPSIS      | 使用方法大纲                 |
| CONFIGURATION | 配置xxx                      |
| DESCRIPTION   | 功能说明                     |
| OPTIONS       | 可选参数说明                 |
| EXAMPLE       | 实例                         |
| EXIT STATUS   | 退出状态（返回给父进程的值） |
| RETURN VALUE  | 返回值                       |
| ERRORS        | 错误类型                     |
| ENVIRONMENT   | 环境变量                     |
| FILES         | 相关配置文件                 |
| VERSIONS      | 版本                         |
| CONFORMING TO | 符合的规范                   |
| NOTES         | 注意事项                     |
| BUGS          | 已经发现的 bug               |
| AUTHORS       | 作者                         |
| SEE ALSO      | 与之功能相近的其它命令       |

man 中的快捷

名称    | 用法
---     |---
/string |	向“下”搜寻string这个字串
?string |	向“上”搜寻string这个字串
n       |	继续下一个搜寻
N       |	反向查询 

与 man 相似的命令是 info，而`info` 手册页按照节点（node）组织的，每个手册页文件是一个节点，手册页内支持链接到其它节点，如此组织犹如一张网，和网页类似。

自定义软件安装路径配置 man page。

```sh
自定义的软件没有安装在 /usr/local/ 路径时，需手动配置 man page，否则使用 man  去查找软件相关的手册时，会找不到。例如：你安装的软件放置到
/usr/local/software/， 那么 man page 搜寻的设置中， 可能就得要在 /etc/man_db.conf 内的 40~50 行左右处， 写入 MANPATH_MAP /usr/local/software/bin /usr/local/software/man，这样才可以使用 man 来查询该软件的在线文件。 
```



# 30. ntsysv

ntsysv 是 CentOS 下图形界面查看系统中有哪些启动的项。


# 31. wget

支持断点下载功能，同时支持FTP和HTTP下载方式，支持代理服务器设置。wget 下载单个文件下载。下载的过程中会显示进度条，包含（下载完成百分比，已经下载的字节，当前下载速度，剩余下载时间）。


参考: [wget命令详解](https://www.cnblogs.com/zhoul/p/9939601.html)



# 32. SHA
- SHA 是安全散列算法（英语：Secure Hash Algorithm，缩写为SHA），它是一个密码散列函数家族，是FIPS所认证的安全散列算法。能计算出一个数字消息所对应到的，长度固定的字符串（又称消息摘要）的算法。且若输入的消息不同，它们对应到不同字符串的机率很高。

- SHA家族的五个算法，分别是SHA-1、SHA-224、SHA-256、SHA-384，和SHA-512，由美国国家安全局（NSA）所设计，并由美国国家标准与技术研究院（NIST）发布，是美国的政府标准。后四者有时并称为SHA-2。SHA-1在许多安全协定中广为使用，包括TLS和SSL、PGP、SSH、S/MIME和IPsec，曾被视为是MD5（更早之前被广为使用的杂凑函数）的后继者。

- 生成 hash 校验: `sha1sum filename`
  ```bash
  #  直接生成 hash 校验后的结果
  Tim@computer:~/Downloads$ sha1sum feeds-master.zip 
  751420b576570fcbfb24e80e47e18168342541e0  feeds-master.zip
  ```
  feeds-master.zip 文件生成的 hash1 校验码为 `751420b576570fcbfb24e80e47e18168342541e0`。

- 为了检验 hash 结果的值是否真确，需要对 hash 结果进行校验。可以将生成 hash 的校验值存入到一个文件中，方便校验，对生成的结果进行校验时，需要加 `-c(check)` 参数。
  ```bash
  Tim@computer:~/Downloads$ sha1sum feeds-master.zip > a.txt
  Tim@computer:~/Downloads$ sha1sum -c a.txt 
  feeds-master.zip: OK
  ```

# 33. md5

md5 是 消息摘要算法（英语：MD5 Message-Digest Algorithm），一种被广泛使用的密码散列函数，可以产生出一个128位（16字节）的散列值（hash value），用于确保信息传输完整一致。MD5由美国密码学家罗纳德·李维斯特（Ronald Linn Rivest）设计，于1992年公开，用以取代MD4算法。

MD5 校验的用法与 SHA 校验的用法一样。下面是 MD5 校验的用法
```
Tim@computer:~/Downloads$ md5sum feeds-master.zip > md.txt
Tim@computer:~/Downloads$ cat md.txt 
f273a8295e2c28e598764ed04898a742  feeds-master.zip
Tim@computer:~/Downloads$ md5sum -c md.txt 
feeds-master.zip: OK
```


# 34. ldconfig

ldconfig 是一个动态链接库管理命令，其目的为了让动态链接库为系统所共享。

主要是在默认搜寻目录 `/lib` 和 `/usr/lib` 以及动态库配置文件 `/etc/ld.so.conf` 内所列的目录下，搜索出可共享的动态链接库（格式如 `lib*.so*`），进而创建出动态装入程序(`ld.so`)所需的连接和缓存文件，缓存文件默认为 `/etc/ld.so.cache`，此文件保存已排好序的动态链接库名字列表。linux下的共享库机制采用了类似高速缓存机制，将库信息保存在 `/etc/ld.so.cache`，程序链接的时候首先从这个文件里查找，然后再到 `ld.so.conf` 的路径中查找。为了让动态链接库为系统所共享，需运行动态链接库的管理命令 `ldconfig`，此执行程序存放在 `/sbin` 目录下。


# 35. ldd

作用：判断某个可执行的二进制文件含有什么动态库。

```sh
[root@zk_190 etc]# ldd -v /usr/bin/cat
        linux-vdso.so.1 =>  (0x00007ffe11572000)
        libc.so.6 => /lib64/libc.so.6 (0x00007f8996748000)
        /lib64/ld-linux-x86-64.so.2 (0x00007f8996b16000)

        Version information:
        /usr/bin/cat:
                libc.so.6 (GLIBC_2.3) => /lib64/libc.so.6
                libc.so.6 (GLIBC_2.3.4) => /lib64/libc.so.6
                libc.so.6 (GLIBC_2.14) => /lib64/libc.so.6
                libc.so.6 (GLIBC_2.4) => /lib64/libc.so.6
                libc.so.6 (GLIBC_2.2.5) => /lib64/libc.so.6
        /lib64/libc.so.6:
                ld-linux-x86-64.so.2 (GLIBC_2.3) => /lib64/ld-linux-x86-64.so.2
                ld-linux-x86-64.so.2 (GLIBC_PRIVATE) => /lib64/ld-linux-x86-64.so.2
        /lib64/ld-linux-x86-64.so.2 (0x00007faf69eef000)

      # 参数 -v 表示该函数来自于哪一个软件
```

# 36. chkconfig

chkconfig 命令用来更新（启动或停止）和查询系统服务的运行级信息。谨记 chkconfig 不是立即自动禁止或激活一个服务，它只是简单的改变了符号连接。



# 37. LD_LIBRARY_PATH

`LD_LIBRARY_PATH` 是 Linux 下用来处理环境变量的，告诉加载器（loader）在什么路径下去查找非标准库中的共享库。

Linux 运行的时候，是如何管理共享库(*.so)的？

​	在 Linux 下面，共享库的寻找和加载是由 /lib/ld.so 实现的。 ld.so 在标准路经(/lib, /usr/lib) 中寻找应用程序用到的共享库。但是，如果需要用到的共享库在非标准路经，ld.so 怎么找到它呢？

目前，Linux 通用的做法是将非标准路经加入 /etc/ld.so.conf，然后运行 ldconfig 生成 /etc/ld.so.cache。 ld.so 加载共享库的时候，会从 ld.so.cache 查找。传统上，Linux 的先辈 Unix 还有一个环境变量：LD_LIBRARY_PATH 来处理非标准路经的共享库。ld.so 加载共享库的时候，也会查找这个变量所设置的路经。

LD_LIBRARY_PATH 的设置方法：用 `export` 命令来设置值

```sh
// 将 /home/John/IOV/Libs 中的共享库路径添加到环境变量中
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/John/IOV/Libs
```

参考

- [LD_LIBRARY_PATH](http://www.cppblog.com/toMyself/archive/2010/08/02/121971.html)
- https://www.csdndocs.com/article/2589063
- [linux 添加动态链接库路径](https://blog.csdn.net/liu0808/article/details/79012187)

# 38. scp 

SCP(secure copy) 是基于ssh协议的安全拷贝，用于将文件/目录安全地从本地主机传输到远程主机。

一般情况下 Linux 服务器都有 scp 命令，如果没有，可通过如下方式安装：

```sh
// centos
yum -y install openssh-clients

// Ubuntu
apt-get install openssh-client 
```

- 复制文件/目录到远程主机

```sh
// 复制文件
scp source_file_name user@destination_host:destination_folder
//复制目录 
scp -r source_directory user@destination_host:destination_folder 

[root@Client ~]# scp text.txt root@192.168.20.40:/root
```

- 从远程主机复制文件/目录

```sh
// 复制文件
scp user@source_host:source_file_name local_destination_folder 
 // 复制目录
scp -r user@source_host:source_file_name local_destination_folder

[root@Client ~]# scp root@192.168.20.40:/root/test40.txt /root
root@192.168.20.40's password: 
test40.txt                                                                                                                   100%   12     4.2KB/s   00:00    
[root@Client ~]# ll | grep test40.txt
-rw-r--r--   1 root    root          12 7月   6 09:41 test40.txt
```



# 39. rsync

Rsync (remote synchronize) 实现同步本地主机和远程主机的文件/目录，和 SCP 不同之处在于，首次复制时，Rsync 会复制整个目录，在后面的复制中，不会复制相同的内容，只对差异文件做更新，scp 是把所有文件都复制过去。Rsync 广泛用于备份和镜像。

安装 Rsync

```sh
// centos
yum install rsync

// Ubuntu
apt-get install rsync  
```

参数项

| 参数        | 功能                                                         |
| ----------- | ------------------------------------------------------------ |
| -t          | 将源文件的修改时间(modify time)同步到目标机器                |
| -I          | --ignore-times，不跳过时间和大小都匹配的文件，也就是不检查是否有改动，直接复制 |
| -r          | 递归，用于目录复制                                           |
| -a(archive) | 存档模式，保存所有的元数据，比如修改时间（modification time）、权限、所有者等，并且软链接也会同步过去。 |
| -v          | 打印复制过程                                                 |
| -l          | 拷贝符号连接                                                 |
| --delete    | 删除目标目录中多余的文件，也就是保持两个目录相同，使得目标目录成为源目录的镜像副本 |

- 复制文件/目录到远程主机。如果复制的目标目录不存在，会自动创建，语法格式和SCP一样：

```sh
// 复制文件
rsync source_file_name/ user@destination_host:destination_folder

// 复制目录
rsync -r source_file_name/ user@destination_host:destination_folder 

[root@Client ~]# rsync test.txt root@192.168.20.40:/root
root@192.168.20.40's password: 
[root@Client ~]# 
[root@Client ~]# rsync -rvl test/ root@192.168.20.40:/root/test222
root@192.168.20.40's password: 
sending incremental file list
created directory /root/test222
./
test2.txt
test40.txt

sent 187 bytes  received 93 bytes  62.22 bytes/sec
total size is 12  speedup is 0.04
```

- 从远程主机复制文件/目录

```sh
// 复制文件
rsync user@source_host:source_file_name local_destination_folder 

// 复制目录
rsync -r user@source_host:source_file_name local_destination_folder 

[root@Client ~]# rsync root@192.168.20.40:/root/test40.txt /root
root@192.168.20.30's password: 
[root@Client ~]# ll test40.txt
-rw-r--r-- 1 root root 12 7月   8 11:11 test40.txt
```

- 排除多个文件或目录

```sh
rsync -avP --exclude={del_file1, del_file2, ...} source_dir dest_dir

rsync -avp --exclude={log,*.so} * root@192.168.153.222:/home/zhoushuhui/IOV/Server
```

参考
- [使用SCP或Rsync实现Linux主机之间文件、目录的复制 | HiYong (hiyongz.github.io)](https://hiyongz.github.io/posts/linux-copying-files-using-scp-or-rsync/)



# 40. 防火墙

## 40.1. ubuntu 下默认的防火墙

- `sudo ufw status` 查看防火墙当前状态
- `sudo ufw enable` 开启防火墙
- `sudo ufw disable` 关闭防火墙
- `sudo ufw version` 查看防火墙版本
- `sudo ufw default allow` 默认允许外部访问本机
- `sudo ufw default deny` 默认拒绝外部访问主机
- `sudo ufw allow 53` 允许外部访问53端口
- `sudo ufw deny 53` 拒绝外部访问53端口
- `sudo ufw allow from 192.168.0.1` 允许某个IP地址访问本机所有端口


## 40.2. CentOS 下默认的防火墙

CentOS7下默认的防火墙为 `firewalld` 
```sh
firwall-cmd：是 Linux 提供的操作 firewall 的一个工具

参数项：
  –-permanent：表示设置为持久；
  –-add-port：标识添加的端口
```

- 启动： systemctl start firewalld
- 关闭： systemctl stop firewalld
- 查看系统防火墙状态： systemctl status firewalld
- 开机禁用 ： systemctl disable firewalld
- 开机启用 ： systemctl enable firewalld
- 查看firewall状态：firewall-cmd --state
- 重启防火墙：firewall-cmd --reload
- 查看版本： firewall-cmd --version
- 查看帮助： firewall-cmd --help
- 查看区域信息: firewall-cmd --get-active-zones
- 查看指定接口所属区域： firewall-cmd --get-zone-of-interface=eth0
- 拒绝所有包：firewall-cmd --panic-on
- 取消拒绝状态： firewall-cmd --panic-off
- 查看是否拒绝： firewall-cmd --query-panic
- 查看开放的端口：firewall-cmd --list-ports
- 查询 `8080` 端口是否开放  firewall-cmd --query-port=8080/tcp
- 开放 `8080` 端口 firewall-cmd --permanent --add-port=8080/tcp
- 移除 `8080` 端口 firewall-cmd --permanent --remove-port=8080/tcp


# 41. SELinux

SELinux 是 Security Enhanced Linux 的缩写，设计的目的是避免资源的利用。SELinux 是在进行进程、文件等详细权限配置时依据的一个核心模块。由于启动网络服务的也是进程，因此刚好也是能够控制网络服务能否存取系统资源的一道关卡。

SELinux 是通过 MAC(Mandatory Access Control：强制访问控制)的方式来管理进程的，它控制的 subject 是进程，object 是该进程能否读取的文件资源。

# 42. QAQ

## 42.1. Linux 与 Windows相差 8 小时

新版本的 Ubuntu 使用 `systemd` 启动之后，时间也改成了由 `timedatectl` 来管理，此方法就不适用了。
`$sudo timedatectl set-local-rtc 1`

重启完成将硬件时间 UTC 改为 CST，双系统时间保持一致。

先在 ubuntu下更新一下时间，确保时间无误：
```bash
$sudo apt-get install utpdate
$sudo ntpdate time.windows.com
```
然后将时间更新到硬件上：`$sudo hwclock --localtime --systohc`

## 编码转换

`enca` 是 Linux 下的文件编码转换工具。

```
安装：apt install enca
查看版本：enca --versioN
```

查看某个文件编码

```
enca -L zh_CN file_name
```

将某个文件转化为 `utf-8`

```
enca -L zh_CN -x utf-8 file_name
```

转化某个文件但如果不想覆盖原文件可以这样

```
enca -L zh_CN -x UTF-8 < file1 > file2 
```

查看当前目录下的文件编码

```
enca -L zh_CN `ls`
```

将当前目录下所有的文件转换为 UTF-8

```
enca -L zh_CN -x utf-8 *
```

源码地址：https://dl.cihar.com/enca/



# 43. Reference

- [Github上Linux工具快速教程](https://github.com/me115/linuxtools_rst) ：这本书专注于Linux工具的最常用用法，以便读者能以最快时间掌握，并在工作中应用
- [如何在centos上安装clang-tidy](https://developers.redhat.com/blog/2017/11/01/getting-started-llvm-toolset/)
- [CentOS 8发布下载，附新功能/新特性介绍](https://ywnz.com/linuxxz/5941.html) 
- [linux ldconfig命令,环境变量文件配置详解](https://blog.csdn.net/winycg/article/details/80572735)
