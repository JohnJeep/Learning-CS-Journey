<!--
 * @Author: JohnJeep
 * @Date: 2020-04-04 09:46:51
 * @LastEditTime: 2021-01-20 09:24:18
 * @LastEditors: Please set LastEditors
 * @Description: Linux基础用法笔记
--> 
<!-- TOC -->

- [1. Linux Basic](#1-linux-basic)
  - [1.1. 基础知识](#11-基础知识)
    - [1.1.1. MBR](#111-mbr)
    - [1.1.2. GPT](#112-gpt)
    - [1.1.3. BIOS与UEFI](#113-bios与uefi)
    - [1.1.4. 分区](#114-分区)
  - [1.2. 文件操作](#12-文件操作)
  - [1.3. 磁盘](#13-磁盘)
  - [1.4. 文件权限与链接](#14-文件权限与链接)
    - [1.4.1. 默认文件权限 umask](#141-默认文件权限-umask)
    - [1.4.2. 文件隐藏属性](#142-文件隐藏属性)
    - [1.4.3. 用户与用户组](#143-用户与用户组)
    - [1.4.4. 用户权限修改](#144-用户权限修改)
    - [1.4.5. 文件种类与扩展名](#145-文件种类与扩展名)
    - [1.4.6. 文件目录](#146-文件目录)
    - [1.4.7. 链接](#147-链接)
  - [1.5. 重定向](#15-重定向)
  - [1.6. wc](#16-wc)
  - [1.7. top](#17-top)
  - [1.8. ps (process status)](#18-ps-process-status)
  - [1.9. pstree](#19-pstree)
  - [1.10. find与grep](#110-find与grep)
  - [1.11. PID](#111-pid)
  - [1.12. 管道](#112-管道)
  - [1.13. 绝对路径与相对路径](#113-绝对路径与相对路径)
  - [1.14. (netstat)查看使用的端口](#114-netstat查看使用的端口)
  - [1.15. dmesg: 分析内核产生的信息](#115-dmesg-分析内核产生的信息)
  - [1.16. vmstat](#116-vmstat)
  - [1.17. 解压与压缩](#117-解压与压缩)
    - [1.17.1. 文件类型](#1171-文件类型)
    - [1.17.2. tar文件打包](#1172-tar文件打包)
      - [1.17.2.1. 打包文件或目录](#11721-打包文件或目录)
      - [1.17.2.2. 解包文件或目录](#11722-解包文件或目录)
      - [1.17.2.3. 解压缩文件](#11723-解压缩文件)
      - [1.17.2.4. 打包压缩](#11724-打包压缩)
    - [1.17.3. zip](#1173-zip)
  - [1.18. man命令](#118-man命令)
  - [1.19. ntsysv](#119-ntsysv)
  - [1.20. strace](#120-strace)
  - [1.21. wget命令](#121-wget命令)
  - [1.22. 包管理](#122-包管理)
    - [1.22.1. 软件仓库](#1221-软件仓库)
    - [1.22.2. apt命令](#1222-apt命令)
    - [1.22.3. dpkg 命令](#1223-dpkg-命令)
  - [1.23. 防火墙](#123-防火墙)
    - [1.23.1. ubuntu下默认的防火墙](#1231-ubuntu下默认的防火墙)
    - [1.23.2. CentOS下默认的防火墙](#1232-centos下默认的防火墙)
  - [1.24. SELinux](#124-selinux)

<!-- /TOC -->

# 1. Linux Basic
- 参考
  - [Github上Linux工具快速教程](https://github.com/me115/linuxtools_rst) ：这本书专注于Linux工具的最常用用法，以便读者能以最快时间掌握，并在工作中应用


## 1.1. 基础知识
- 磁盘阵列（RAID）：利用硬件技术将数个硬盘整合成为一个大硬盘的方法， 操作系统只会看到最后被整合起来的大硬盘。 由于磁盘阵列是由多个硬盘组成， 所以可以达成速度性能、 备份等任务。 


### 1.1.1. MBR
> MBR(Master Boot Record): 主引导记录
- 引导启动程序记录区与分区表通常放在磁盘的第一个扇区，这个扇区通常大小为 512 bytes。
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

- 扩展分区
  - 扩展分区并不是只占一个区块，而是会分布在每个分区的最前面几个扇区来记载分区信息的！
  - 扩展分区的目的是使用额外的扇区来记录分区信息， 扩展分区本身并不能被拿来格式化。 然后我们可以通过扩展分区所指向的那个区块继续作分区的记录。

- MBR 主要分区(Primary)、 扩展分区(Extend)与逻辑分区(logical) 三者的区别
  - 主要分区与扩展分区最多可以有四个（ 硬盘的限制）
  - 扩展分区最多只能有一个（ 操作系统的限制）
  - 逻辑分区是由扩展分区持续切割出来的分区；
  - 能够被格式化后作为数据存取的分区是：主要分区与逻辑分区，扩展分区无法格式化；
  - 逻辑分区的数量依操作系统而不同，在Linux系统中 SATA 硬盘已经可以突破63个以上的分区限制；

- MBR分区的缺点
  - 操作系统无法抓取到 2T 以上的磁盘容量！
  - MBR 仅有一个区块，若被破坏后，经常无法或很难救援。
  - MBR 内存放的启动引导程序最大为 446Bytes， 无法容纳较多的程序码。

### 1.1.2. GPT
> GPT: GUID Partition Table，源自EFI标准的一种较新的磁盘分区表结构的标准，支持64位的寻址。
- LBA(Logical Block Address): 逻辑区块位址
  - LBA0(MBR兼容区块)：储存了第一阶段的启动引导程序。
  - LBA1(GPT表头记录)：记录了分区表本身的位置与大小， 同时记录了备份用的 GPT 分区（在最后 34 个 LBA 区块）放置的位置，同时放置了分区表的检验机制码（ CRC32），操作系统可以根据这个检验码来判断 GPT 是否正确。 
  - LBA2-33（实际记录分区的信息地方）：从 LBA2 区块开始，每个 LBA 都可以记录 4 组分区记录，所以在默认的情况下，总共可以有 4*32 = 128 组分区记录。 因为每个 LBA 有 512Bytes， 因此每组记录用到 128Bytes 的空间， 除了每组记录所需要的识别码与相关的记录之外， GPT 在每组记录中分别提供了 64bits 来记载开始/结束的扇区号码。

> GPT 分区已经没有所谓的主、 扩展、 逻辑分区的概念，既然每组纪录都可以独立存在，当然每个都可以视为是主分区， 每一个分区都可以拿来格式化使用。


### 1.1.3. BIOS与UEFI
- BIOS：是一个写入到主板上的一个软件程序（仅有16位），采用汇编语言编写的。 
- Boot loader的主要任务
  - 提供加载项：用户可以选择不同的启动选项，这也是多重引导的重要功能。
  - 加载内核文件：直接指向可使用的程序区段，来启动操作系统。
  - 转交其它启动引导程序：将启动管理功能转交给其它引导程序负责。
> 每个分区都有自己的启动扇区(boot sector)。启动引导程序只会认识自己的系统分区内的可开机核心文件， 以及其它启动引导程序而已；

> 如果要安装多重开机，为什么最好先安装Windows再安装Linux呢？
>
> 因为 Linux在安装的时候，你可以选择将启动引导程序安装在 MBR 或其它分区的启动扇区， 而且Linux的启动引导程序可以手动设置选项，所以你可以在Linux的启动引导程序里面加入Windows启动的选项；
Windows在安装的时候， 它的安装程序会主动的覆盖掉 MBR 以及自己所在分区的启动扇区，你没有选择的机会， 而且它没有让我们自己选择选项的功能。因此，如果先安装Linux再安装Windows的话，那么 MBR 的启动引导程序就只会有Windows的选项， 而不会有Linux的选项（ 因为原本在MBR内的Linux的启动引导程序就会被覆盖掉） 。

- UEFI(Unified Extensible Firmware Interface): 统一可扩展固件接口，采用C语言编写的。其中UEFI可以直接获取GPT的分区表。

> 为什么在安装系统时，需要将UEFI的secure boot关闭？
> 
> 因为使用secure boot会使将要启动的操作系统，必须要被UEFI验证，否则就无法启动。

### 1.1.4. 分区
- 挂载：利用一个目录当成进入点，将磁盘分区的数据放置在该目录下。其中根目录（/）必须挂载到某个分区，其它的目录可以依据用户的的需求挂载到不同的分区。

- 操作系统开机的流程：BIOS--->MBR--->引导启动程序--->内核文件
- swap分区：磁盘模拟内存的交换分区，当有数据被存放在物理内存里面，但这些数据又不是常被CPU使用时，这些不常被使用的数据将会被扔到硬盘的交换分区当中去，而速度较快的物理内存将被释放出来给真正需要的程序使用。交换分区不会使用目录树的挂载，所有交换分区就不需要指定挂载点。



## 1.2. 文件操作
- `touch fileName`: 文件不存在新建一个文本；文件存在时，修改文件创建的时间。
- `rm fileName(remove)`: 删除一个文本
- `mkdir(make directory) `: 新建一个目录，创建多层的目录加参数 `-p` (--parents)
- `rmdir `: 移除一个目录
- `rm -rf `: 删除一个目录下的所有文件；以下为两个常用的参数
  - `-i(interactive)`：让系统在执行前确认。
  - `-r(recursive)`：表示递归
- `mv  -v A B`: 文件A移到B处，并重名为B，`-v`显示系统执行的操作(move)。
- `cp src dest`: 将文件src拷贝到当前目录下为dest文件；
  - 注意：拷贝目录时，要加 `-r` 参数(recursive)
  - 拷贝操作会复制执行者的属性和权限。
- `ls -al`: 查看所有隐藏的文件  
- `uname -a`: 查看Linux版本
- `lscpu`: 查看系统CPU情况
- `locale -a`： 列出系统支持的所有语言环境
- `nslookup 域名` 查看域名对应的IP地址
- `which`
  - 在 `PATH` 变量指定的路径中，搜索某个系统命令的位置，并且返回第一个搜索结果。
  - 示例：`which gcc`
- `whereis`: 查找系统中包含可以找到的所有文件
  - 只能用于搜索程序名和二进制文件（参数-b）
  - 示例：`whereis gcc`
- `eject`: 将光盘驱动器中的光盘轻轻弹出和收回


- 查看文件内容
  - `cat(concatenate)`：从第一行开始显示文件内容，将要显示的内容一次性的输出在屏幕上。
  - `tac`：从最后一行开始显示文件内容。
  - `nl` ：查看文件时可以显示行号。
  - `more`：(`-n`：可以显示行号)一页一页的显示文件内容，只能往后翻。
  - `less`：(`-n`：可以显示行号)一页一页的显示文件内容，既可以往后翻又可以往前翻。一般用的最多。
    - 空格键：向下翻一页。一般使用上下箭头进行翻页。
    - /字符串：向下查找字符串。
    - ?字符串：向上查找字符串。
    - n：重复前一个查找。
    - N：反向重复前一个查找。
    - g：进到这个数据的第一行。
    - G：进到这个数据的最后一行。
    - q：退出less程序。
  - `od(Octal Dump)`：默认以二进制的方式读取文件内容。
    - 将指定文件内容以八进制、十进制、十六进制、浮点格式或 ASCII 编码字符方式显示，通常用于显示或查看文件中不能直接显示在终端的字符。
    - 格式：`od -t TYPE` 文件
    - 参数
      ```
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
        
        SIZE 可以为数字，也可以为大写字母。如果 TYPE 是 [doux] 中的一个，那么 SIZE 可以为 C  = sizeof(char)，S = sizeof(short)，I = sizeof(int)，L = sizeof(long)。如果 TYPE 是 f，那么 SIZE 可以为 F = sizeof(float)，D = sizeof(double) ，L = sizeof(long double)
      ```
    - 示例
      - `od -t x testfile`，以十六进制输出testfile，默认以四字节为一组（一列）显示。
      - 利用 od 命令来查看字符的ASCII表：`echo abc | od -t dCc`

  - head、tail：取出文件前几行或最后几行的数据。
    > 示例：在屏幕上列出 `/etc/man_db.conf` 文件中的第11行到22行之间的内容，并且显示行号。 `cat -n /etc/man_db.conf | head -n 20 | tail -n 10`



## 1.3. 磁盘
- `du(disk usage)`: 显示指定的目录或文件所占用的磁盘空间大小
  - `-h(human)`：以人类容易看懂的方式显示
  - `-M(megabytes)`: 以1MB为单位
  - `-s(summarize)`: 仅显示总计
  - 示例：`du -h`

- `df(disk free)`: 显示Linux 系统上文件系统的磁盘使用情况 
  - `-h(human)`：以人类容易看懂的方式显示
  - `-i(inodes)`: 列出 inode 信息，不列出已使用的 block
  - `-H`: 很像 -h, 但是用 1000 为单位而不是用 1024

- `fdisk -l` 查看硬盘的情况

- `mount 设备名字 挂在目录`
  - `mount` 设备装载常用命令：`mount –t type dev dir`
  - `–t type` 是需要挂载的文件系统类型，光盘文件系统类型是：iso9660；
  - `dev` 是挂载文件系统的设备名称，光盘驱动器的设备名称是/dev/cdrom; 
  - `dir`表示挂载点，即挂载到的文件目录路径
  - 例如：`mount -t iso9660 /dev/cdrom /media/drom`
- `umout` 设备装载常用命令
   - 例如：`umount dir device […]`


## 1.4. 文件权限与链接
> CentOS使用的是 xfs 作为默认的文件系统。

文件权限分为
  - 可读（Read），可以读取文件的内容。
  - 可写（Write），可以编辑、新增、或修改该文件的内容，但不具备删除该文件的权限。
  - 可执行（eXecute），Linux下，文件是否能够执行，与文件的后缀名无关，仅由是否具备 `x` 这个权限来决定。

> 注意: X 代表这个文件具有可执行的能力，但能不能执行成功，需要由文件中的内容决定。

> 对于文件的rwx来说，主要都是针对“文件的内容”而言，与文件文件名的存在与否没有关系，因为文件记录的是实际的数据。文件是存放数据的所在，目录则主要记录文件名列表。因此文件名与目录有强烈的关联。


目录权限
  - r: 具有读取文件目录结构的权限
  - w: 具有改动该目录结构列表的权限。
  - x: 目录不能被执行，x 表示用户能否进入该目录并且成为工作目录。
  > 通常一个用户给其它的用户开放目录，至少要具备 `rx` 权限，其它的用户才能访问当前用户的目录。


### 1.4.1. 默认文件权限 umask
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


### 1.4.2. 文件隐藏属性
- 修改文件的隐藏属性：`chattr [+-=] [ASacdistu]` 该命令一般用于对数据的安全性比较高的地方
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

- 查看文件或目录的隐藏属性：`lsattr [-adR] 文件或目录`

### 1.4.3. 用户与用户组
说明 | 用户(owner) | 用户组(group) | 其它用户(other)
--- | --- | --- |---
权限 | 读  写  执行 | 读  写  执行| 读  写  执行
符号 | r  w  x | r  w  x | r  w  x
权值 | 4 2 1 | 4 2 1 | 4 2 1 


`ls -l`: 查看目录下文件属性的所有信息，每一栏说明如下：
  - 第1栏有10个字符，其中第1个字符描述类型，后边9个字符描述权限
  - 第2栏是硬链接数，删除文件其实是将链接数减1 ，减到0了就真正删除文件内容
  - 第3、4栏分别是文件的拥有者及所属群组。
  - 第5栏是文件大小，单位为字节（Byte）
  - 第6栏是最近访问（修改）时间
  - 第7栏是文件名，对于符号链接


### 1.4.4. 用户权限修改
- `chown`: 修改文件的拥有者（用户）
- `chgrp`: 修改文件的用户组
- `chmod`: 修改文件的模式权限，SUID、SGID、SBIT等特性

特殊权限
- SUID(Set UID)，简写：s，数字：4
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

> 利用符号的方式修改文件权限：SUID为 u+s，SGID为 g+s，SBIT为 o+t



### 1.4.5. 文件种类与扩展名
类型符号 | 文件类型 | 分类
---|--- | ---
- | 一般文件(regular)  | 纯文本文件、二进制文件、数据文件
d | 目录文件(directory) | null
c | 字符设备文件(character device) | null
l | 符号链接文件(symbolic link) | null
p, f | 数据传输文件(pipe, FIFO) | null
s | 套接文件(socket) | null

> Linux下文件的最大长度：单一文件或目录的最大允许文件名为 255bytes，以一个ASCII英文占用一个字节来说，则可达255个字符长度，每个汉字占用 2个字节，则最大的文件名大约在 128 个汉字之间。


### 1.4.6. 文件目录
> FHS(Filesystem Hierarchy Standard)：文件系统分层标准
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
-  sys：这个目录其实跟/proc非常类似，也是一个虚拟的文件系统，主要也是记录核心与系统硬件信息较相关的信息。 包括目前已载入的核心模块与内核侦测到的硬件设备信息等等。这个目录同样不占硬盘容量。
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

```
一些链接文件之间的关系

/bin --> /usr/bin
/sbin --> /usr/sbin
/lib --> /usr/lib
/lib64 --> /usr/lib64
/var/lock --> /run/lock
/var/run --> /run
```



### 1.4.7. 链接
> `ln`: 建立链接命令
-  `ln -s source  destination` 软连接
- `ln 原文件 新文件 ` 硬链接: 指向磁盘中文件的节点(inode),只有文件才能创建硬链接，目录不能创建。


## 1.5. 重定向 
- `>` 重定向：`ls > test.txt` 将`ls`列出的所有文件放到test.txt中，并`覆盖原`来test.txt文件中的内容
- `命令 >> 文件名` 例如：`pwd >> text.tct`将用`pwd`生成的数据放到`test.txt`文件中，`不会覆盖原`来的文件，新加的文件保留到文本后面。



## 1.6. wc
- 作用：print newline, word, and byte counts for each file.用来计算一个文件或者指定的多个文件中的行数，单词数和字符数。
- 选项参数
  - `-c`: 打印字节数
  - `-l`: 打印列数
  - `-w`:  打印word counts


## 1.7. top
- 动态查看进程的变化，默认按照CPU使用率为排序的依据。可以在系统的后台执行，得到进程的全部信息。
- 使用：`top -bn 1 -i -c` 
- 参数 
  - `%us` 表示用户空间程序的cpu使用率（没有通过nice调度）
  - `%sy` 表示系统空间的cpu使用率，主要是内核程序。
  - `%ni` 表示用户空间且通过nice调度过的程序的cpu使用率。
  - `%id` 空闲cpu
  - `%wa` cpu运行时在等待io的时间
  - `%hi` cpu处理硬中断的数量
  - `%si` cpu处理软中断的数量
  - `%st` 被虚拟机偷走的cpu 


## 1.8. ps (process status)
- 显示格式参数
  - `USER `：用户名
  - `%CPU `：该进程用掉的CPU百分比
  - `%MEM `：进程占用内存的百分比
  - `VSZ  `：该进程使用的虚拟內存量（KB）
  - `RSS  `：该进程占用的固定內存量（KB）（驻留中页的数量）
  - `TTY  `：表示该进程在那个终端上运行，若与终端无关，则显示? 若为pts/n，则表示由网络连接进入主机的进程，tty1-tty6 表示是本机上面的登录进程。
  - `STAT `：该进程目前的状态
  - `START`：该进程被触发启动的时间
  - `TIME `：该进程实际使用CPU运行的时间
  - `CMD(command)`：执行此进程触发的命令是什么
  - `UID  `：用户ID、但输出的是用户名
  - `PID  `：进程的ID
  - `PPID `：父进程ID
  - `C    `：CPU使用率，单位为百分比
  - `STIME`：进程启动到现在的时间
  - `PRI(priority)`：进程被CPU执行的优先级，数值越小，代表该进程被CPU执行的越快。这个值由内核动态调整，用户无法直接调整PRI的值。
  - `NI(nice)`：调整进程的优先级。
    - nice值的可调整的范围在 -20~19 之间。
    - root用户可以随意调整自己或其它用户进程的 nice值，且范围范围在 -20~19 之间。
    - 一般用户只能调整自己进程的nice值，范围仅为 0-19，避免一般用户去抢占系统的资源。
    - PRI与NI之间的关系：`PRI(new) = PRI(old) + nice`
    - nice值有正负，当nice值为负数时，那么该进程会降低PRI的值，会变得较优先处理。
    - 如何调整nice值？
      - 进程刚开始时就给指定一个特定的nice值。`nice -n -5 vim &` 启动vim时，给定一个nice值，并将vim放在后台执行。
      - 调整已存在的进程的nice值，需要用 `renice` 命令：`renice 4 2366` 将PID=2366进程的nice值调整为4
  - `ADDR`：是内核函数，指出该进程在内存中的哪个部分；如果是个running的进程，一般用 `-` 表示
  - `SZ`：表示此进程用掉多少内存
  - `WCHAN`：表示目前进程是否在运行，如果为 `-`，则表示正在运行。


- STAT状态位常见的状态字符
  - `D` 无法中断的休眠状态（通常 IO 的进程）；
  - `R(running)` 正在运行可中在队列中可过行的；
  - `S(大写：sleep)` 处于休眠状态；
  - `T(stop)` 停止或被追踪状态；
  - `W` 进入内存交换 （从内核2.6开始无效）；
  - `X` 死掉的进程 （基本很少见）；
  - `Z(zombie)` 僵尸状态，进程已被终止，但无法被删除；
  - `<` 优先级高的进程
  - `N` 优先级较低的进程
  - `L` 有些页被锁进内存；
  - `s` 进程的领导者（在它之下有子进程）；
  - `l` 多线程，克隆线程（使用 CLONE_THREAD, 类似 NPTL pthreads）；
  - `+` 位于后台的进程组；


- `ps –ef|grep 程序名称`：查看一个程序是否运行 
- `ps -Lf 端口号|wc -l `：查看线程个数  
- `ps -l`：查看当前用户的bash进程
- `ps aux`：查看系统运行的所有进程，默认按照PID的顺序排序。
- `ps axjf`：查看系统运行的所有进程，并带有PPID项


## 1.9. pstree 
- 查找各个进程之间的相关性
- Linux系统中内核调用的第一个进程为 `systemd`，该进程的PID为 `1`



## 1.10. find与grep
- find：按照文件属性查找 
  - 选项参数
    - `-name`: 例子：find /usr/src -name filename.txt
    - `-type`: 例子：find /usr/src -type filename.txt
      > 类型包括：f(file), d(directory), l(link), c(char), d(device), s(socket), b(block)
    - `-size`: 默认单位为512byte，一个扇区的大小
      - 例子：find /usr/src -size +10M -size -20M 查找大于10M小于20M的文件
      - 例子：find /usr/src -size +10k -size -20k
    - `-maxdepth`: 例子： find /usr -maxdepth 2 -type d | wc -l  统计/usr 目录下深度为2的所有目录文件
    - `-exec`: 例子： find ./ -name "*.sh" -exec ls -l {} \;  列出当前目录下所有的 .sh 文件，并执行ls -l 命令
    - `-print`: 将文件或目录名称列出到标准输出。格式为每列一个名称，每个名称前皆有“./”字符串；
    - `print0`: 就将文件或目录名称列出到标准输出。格式为全部的名称皆在同一行；  
    - `xargs`
      -  以空格或`\0` 作为分隔符拆分解析的命令。每次从缓冲区读取的数据有限，而exec则是一次性将查到的内容读到缓冲区内。
      - 例子：find ./ -name "*.sh" -print | xargs ls -l  结合管道一起使用，用法与exec一样。 
    - `-atime(access time)`: 访问时间， `+7` 七天以前，`-7` 最近七天以内访问过的
    - `-amin`: 访问时间（按照分钟）
    - `-mtime`: 上次修改的时间（按照天数）
    - `-mmin(modified minute) `: 修改时间（按照分钟）
    - `ctime(change time)`: 最近文件的状态被改变的时间
    - `cmin(change minute)`: 最近文件的状态被改变的时间（按照分钟）


- `grep(global search regular expression and print out the line)` 全面搜索正则表达式和打印输出行 
  - 三种形式的grep命令
    - gerp:标准格式
    - egrep:扩展grep命令，其实和grep -E等价，支持基本和扩展的正则表达式。
    - fgrep: 快速grep命令，其实和grep -F等价，不支持正则表达式，按照字符串表面意思进行匹配。
  - 选项参数
    - `-n`：列出所有的匹配行，显示行号
    - `-r`: 递归搜索
    - `-R`：  
    - `-i`: 搜索时，忽略大小写
    - `-c`: 只输出匹配行的数量
    - `-l`: 只列出符合匹配的文件名，不列出具体的匹配行
    - `-h`: 查询多文件时不显示文件名
    - `-s`: 不显示不存在、没有匹配文本的错误信息
    - `-v`: 显示不包含匹配文本的所有行
    - `-w`: 匹配整个单词
    - `-x`: 匹配整行
    - `-q`: 禁止输出任何结果，已退出状态表示搜索是否成功
    - `-b`: 打印匹配行距文件头部的偏移量，以字节为单位
    - `-o`: 与-b结合使用，打印匹配的词据文件头部的偏移量，以字节为单位
    ```
    grep -r  "task_struct {" /usr/src/  -n       搜索 /usr/src/ 目录下包含 task_struct { 的字符，并显示字符所在的行号
    ```


## 1.11. PID
- `pgrep -l xxxx(程序名称)`  只显示某个进程的PID
- `ps aux | grep xxx(程序名称)`  显示某个进程的全部信息，包括PID
- `ps ajx` 显示进程组ID
- `ulimit -a` 查看资源的上限大小 
- `pidof program_name`：找出某个正在执行的进程的PID




## 1.12. 管道
> 定义：将一个命令的输出传送给另一个命令，作为另一个命令的输入，常用 `|` 表示。管道常与grep命令组合使用：`grep 命令1|命令2|命令3|·····|命令n`

- &和&&  |和||四者区别
  - `& `: 表示将当前任务放在后台执行，如要在后台运行redis-server,则有  redis-server &
  - `&&`: 表示前一条命令执行成功时，才执行后一条命令 ，如 echo 'hello‘ && echo 'world'    
  - `| `: 表示管道，上一条命令的输出，作为下一条命令参数，如 echo 'hello' | wc -l
  - `||`: 表示上一条命令执行失败后，才执行下一条命令，如 cat nofile || echo "failed"


## 1.13. 绝对路径与相对路径
- 绝对路径：一定由跟目录(`/`)写起。例如：`/usr/share/doc`  在shell脚本中一般使用绝对路径，防止因为不同的工作环境导致一些问题的发生。
- 相对路径：不是由根目录(`/`)写起。例如：`../man`  相对路径只是相对于当前的工作路径。



## 1.14. (netstat)查看使用的端口
- 侦听端口：应用程序或进程侦听的网络端口，充当通信端点
- 同一个 IP 地址上不能用两个不同的服务去侦听同一端口
- `netstat` 检查端口
  - ` -t `  显示 TCP 端口。
  - ` -u `  显示 UDP 端口。
  - ` -n `  显示数字地址而不是主机名。
  - ` -l `  仅显示侦听端口。
  - ` -p `  显示进程的 `PID和名称`
    - ` netstat -tunlp ` 列出正在侦听的所有TCP或UDP端口
    - Proto - 套接字使用的协议。
    - Local Address - 进程侦听的 IP 地址和端口号。
    - PID/Program name  - PID 和进程名称
  
- 查看指定端口号的所有进程在TCP、UDP传输中的所有状态
  - `netstat -apn | grep 端口号`

- `ss` 检查端口
  - `ss -tunlp`

- `lsof` 检查端口
  - `lsof -nP -iTCP -sTCP:LISTEN ` 获取所有侦听 TCP 端口的列表 


## 1.15. dmesg: 分析内核产生的信息
系统在启动的时候，内核会去检测系统的硬件，你的某些硬件到底有没有被识别，就与这个时候的侦测有关。 但是这些侦测的过程要不是没有显示在屏幕上，就是很飞快的在屏幕上一闪而逝。能不能把内核检测的信息识别出来看看？ 可以使用 dmesg 。所有内核检测的信息，不管是启动时候还是系统运行过程中，反正只要是内核产生的信息，都会被记录到内存中的某个保护区段。 dmesg 这个指令就能够将该区段的信息读出来。


## 1.16. vmstat
可以检测系统资源（CPU、内存、磁盘、IO状态）的变化。


## 1.17. 解压与压缩
### 1.17.1. 文件类型
- `gzip`: 压缩文件后缀(*.gz)
- `bzip2`: 压缩文件后缀(*.bz2)
- `xz`: 压缩文件后缀(*.xz)
<br>
<img src="./pictures/compress.png">



### 1.17.2. tar文件打包     
- 参数
	- `-c(Create)`: 打包文件
	- `-t(lisT)`: 察看打包文件的内容含有哪些文件名
	- `-x(eXtract)`: 解压打包文件 
      > 注意：`-c, -t, -x` 不可同时出现在一串命令行中。
	- `-v(Verbose)`: 在压缩/解压缩的过程中，将正在处理的文件名显示出来
	- `-f(Filename)`:  后面要立刻接要被处理的文件名！
	- `-C(direCtory: 目录)`: 将文件解压在特定的目录
	- `-p(小写)`:  保存原本文件的权限与属性，不包含根目录(/)。
	- `-P(大写)`：保留绝对路径，即允许备份的数据中中包含根目录。解压后的数据直接从根目录(/)开始。
	- `-z`：通过 gzip 的支持进行压缩/解压，此时文件名最好为  `*.tar.gz`
	- `-j`：通过 bzip2 的支持进行压缩/解压，此时文件名最好为 `*.tar.bz2`
	- `-J` ：通过 xz 的支持进行压缩/解压缩：此时文件名最好为 `*.tar.xz`
	   > 注意：`-z, -j, -J` 不可以同时出现在一串命令行中


#### 1.17.2.1. 打包文件或目录
  - 将当前目录下的  anaconda-ks.cfg 文件打包成 A.tar：`tar -cvf A.tar anaconda-ks.cfg `
   - 打包多个文件或目录，中间需要用空格分开：`tar -cvf B.tar anaconda-ks.cfg /tmp/`


#### 1.17.2.2. 解包文件或目录
-  格式：`tar -xvf 解压的文件  -C 文件解压后的路径`
-  注意：若后面不跟 `-C 文件解压后的路径` ，则会默认解包到当前路径下
-  `tar -xvf test.tar -C /tmp` ：将test.tar打包的文件解包到 /tmp 路径下


#### 1.17.2.3. 解压缩文件
- 解多个文件
  > `tar -zxvf etc.tar.gz -C ~/Exercise_Linux/tmp`       将etc.tar.gz文件解压到~/Exercise_Linux/tmp目录下，不加 `-C ~/Exercise_Linux/tmp ` 则只是解压到当前目录下。
  
  > 注意：指定解压的目录必须要首先存在，否则会出错，tar命令不会自动创建不存在的文件夹。

- 解压单一文件
  - `tar -zxvf etc.tar.gz etc/gdb`   将 etc.tar.gz压缩包中gdb文件夹解压 到当前目录下。


#### 1.17.2.4. 打包压缩
- 打包压缩所有文件
  >  `tar -zcvf 自己创建的文件名(xxx.tar.gz)  要打包压缩的路径(/etc/)`: 这样压缩的文件连要压缩文件的目录也一起给压缩了。

- 打包压缩所有文件不包含打包文件的路径
  > `tar -zcvf tmp4.tar.gz -C etc/ .`    将etc下所有文件打包为tmp4.tar.gz 不包含etc包的路径。

- 打包并压缩一个目录，但不含该目录下的某些文件
  > `tar -zcvf bb.tar.gz --exclude=etc/apt etc`   将etc目录下除去apt文件的所有文件打包压缩为 bb.tar.gz，打包压缩时包含etc的路径。
	  
    1.  一定要注意排除目录的最后不要带"/", 否则exclude目录将不起作用
    2.  压缩目录和排除目录都需要采用同样的格式，如都采用绝对路径或者相对路径


### 1.17.3. zip
> zip是压缩指令,unzip是解压指令。zip指令既可以压缩文件，也可以压缩目录。压缩会自动保留源文件，解压会自动保留压缩文件。

- zip -r yasuo.zip demo.txt mydir  // 将demo.txt文件和目录mydir压缩成压缩文件yasuo.zip，选项-r表示递归
- zip -r  mydir.zip  mydir         // 压缩当前目录下的子目录mydir
- unzip   yasuo.zip                // 解压yasuo.zip文件到当前目录
- unzip -d /mydir yasuo.zip        // 把压缩文件解压到指定的mydir目录
- unzip -t  yasuo.zip              // 检查压缩文件是否损坏
- unzip  -l  demo.zip              // 显示demo.zip压缩包中有哪些文件，不进行解压
- unzip  -n  demo.zip              // 解压时不覆盖已存在的文件

> 注意：直接使用unzip指令（不带选项）解压文件时，如果解压文件中包含有文件与当前目录下的某个文件重名，那么会询问是否要覆盖这个文件。



## 1.18. man命令
> man是 POSIX(Portable Operating System Interface) 规定的帮助手册程序。

> `info`手册页按照节点（node）组织的，每个手册页文件是一个节点，手册页内支持链接到其它节点，如此组织犹如一张网，和网页类似。

`man -n 命令参数`：n为数字
- 1：普通应用程序或shell命令
- 2：系统调用
- 3：库函数
- 4：设备文件
- 5：文件格式、或相关协议
- 6：游戏设备
- 7：其它设备
- 8：root管理命令
- 9：非标准的内核程序
<img src="./pictures/man代号.png">

- man手册中的一些关键字
  ```
  NAME - 命令名
  SYNOPSIS - 使用方法大纲
  CONFIGURATION - 配置
  DESCRIPTION - 功能说明
  OPTIONS - 可选参数说明
  EXIT STATUS - 退出状态, 这是一个返回给父进程的值
  RETURN VALUE - 返回值
  ERRORS - 可能出现的错误类型
  ENVIRONMENT - 环境变量
  FILES - 相关配置文件
  VERSIONS - 版本
  CONFORMING TO - 符合的规范
  NOTES - 使用注意事项
  BUGS - 已经发现的bug
  EXAMPLE - 一些例子
  AUTHORS - 作者
  SEE ALSO - 功能或操作对象相近的其它命令
  ```

man中的一些快捷操作:

名称 | 用法
---|---
/string |	向“下”搜寻string这个字串
?string |	向“上”搜寻string这个字串
n |	继续下一个搜寻
N |	反向查询 


## 1.19. ntsysv
- CentOS下图形界面查看系统中有哪些启动的项。


## 1.20. strace
strace  - trace system calls and signals
- 监控用户进程与内核进程的交互
- 追踪进程的系统调用、信号传递、状态变化。


## 1.21. wget命令
- 参考
  - [wget命令详解](https://www.cnblogs.com/zhoul/p/9939601.html)

- 支持断点下载功能，同时支持FTP和HTTP下载方式，支持代理服务器设置
-  wget 下载单个文件下载
  - 下载的过程中会显示进度条，包含（下载完成百分比，已经下载的字节，当前下载速度，剩余下载时间）。


## 1.22. 包管理
- [RedHat/CentOS8 【国内/本地/私有 Yum 源】制作和使用](https://www.jianshu.com/p/68db74388600)

Debian/Ubuntu采用 `dpkg` 进行软件包的管理，使用 `apt` 进行在线软件的升级。

CentOS/Red Hat/Fedora采用 `rpm` 进行软件包的管理，使用 `yum` 进行在线软件的升级。


### 1.22.1. 软件仓库
- 清华大学镜像网站：https://mirrors.tuna.tsinghua.edu.cn/cygwin/
- Windows
  > 常有文件程序自解压、选定安装组件和安装路径（个别不让选择路径）、添加注册表项等等，完成以后双击启动运行，卸载时找安装路径下的uninstall或者控制面板里卸载
- Linux或Unix
   - 软件包组织方式上，是将可执行程序、程序库、手册页等多种类型文件打包压缩提供，内容上一般分为预先编译好的二进制包和程序源码包两种；
   - 软件包管理方式上，不同开发者开发的软件，被打包集中统一存放在官方维护的软件仓库中，这个软件仓库就是一个软件源，和iOS/Android系统上的AppStore/应用市场等概念很像，Windows也开始使用“Windows Store”；除此外，第三方在遵守相关协议的前提下镜像（mirror）官方软件仓库成为镜像源，系统提供专门的软件包管理器用于从软件源下载、安装和卸载软件包。


### 1.22.2. apt命令
- apt-cache search # ------(package 搜索包)
- apt-cache show #------(package 获取包的相关信息，如说明、大小、版本等)
- apt-get install # ------(package 安装包)
- apt-get install # -----(package --reinstall 重新安装包)
- apt-get -f install # -----(强制安装, "-f = --fix-missing"当是修复安装吧...)
- apt-get remove #-----(package 删除包)
- apt-get remove --purge # ------(package 删除包，包括删除配置文件等)
- apt-get autoremove --purge # ----(package 删除包及其依赖的软件包+配置文件等
- `apt-get update`  更新源(软件列表)
- `apt-get upgrade` 更新已安装的包
- `apt-get clean && apt-get autoclean`  清理下载文件的缓存和只清理过时的包
- apt-get dist-upgrade # ---------升级系统
- apt-get dselect-upgrade #------使用 dselect 升级
- apt-cache depends #-------(package 了解使用依赖)
- apt-cache rdepends # ------(package 了解某个具体的依赖,当是查看该包被哪些包依赖吧...)
- apt-get build-dep # ------(package 安装相关的编译环境)
- apt-get source #------(package 下载该包的源代码)
- apt-get check #-------检查是否有损坏的依赖
- `dpkg -S filename` -----查找filename属于哪个软件包
- apt-file search filename -----查找filename属于哪个软件包
- apt-file list packagename -----列出软件包的内容
- apt-file update --更新apt-file的数据库
- 找到安装的软件
  - dpkg -S softwarename 显示包含此软件包的所有位置
  - dpkg -L softwarename 显示安装路径
  - dpkg -l softwarename 查看软件版本
  - 用 find 或 whereis 命令查找文件位置
-  卸载软件
    -  apt-get remove softname1 softname2 …;              移除式卸载
    - apt-get purge sofname1 softname2…;                       卸载软件同时清除配置文件


### 1.22.3. dpkg 命令
- dpkg --info "软件包名" --列出软件包解包后的包名称.
- dpkg -l --列出当前系统中所有的包.可以和参数less一起使用在分屏查看. (类似于rpm -qa)
- dpkg -l |grep -i "软件包名" --查看系统中与"软件包名"相关联的包.
- dpkg -s 查询已安装的包的详细信息.
- dpkg -L 查询系统中已安装的软件包所安装的位置. (类似于rpm -ql)
- dpkg -S 查询系统中某个文件属于哪个软件包. (类似于rpm -qf)
- dpkg -I 查询deb包的详细信息,在一个软件包下载到本地之后看看用不用安装(看一下呗).
- dpkg -i 手动安装软件包(这个命令并不能解决软件包之前的依赖性问题),如果在安装某一个软件包的时候遇到了软件依- 赖的问题,可以用apt-get -f install在解决信赖性这个问题.
- dpkg -r 卸载软件包.不是完全的卸载,它的配置文件还存在.
- dpkg -P 全部卸载(但是还是不能解决软件包的依赖性的问题)
- dpkg -reconfigure 重新配置


## 1.23. 防火墙
### 1.23.1. ubuntu下默认的防火墙
- `sudo ufw status` 查看防火墙当前状态
- `sudo ufw enable` 开启防火墙
- `sudo ufw disable` 关闭防火墙
- `sudo ufw version` 查看防火墙版本
- `sudo ufw default allow` 默认允许外部访问本机
- `sudo ufw default deny` 默认拒绝外部访问主机
- `sudo ufw allow 53` 允许外部访问53端口
- `sudo ufw deny 53` 拒绝外部访问53端口
- `sudo ufw allow from 192.168.0.1` 允许某个IP地址访问本机所有端口


### 1.23.2. CentOS下默认的防火墙
- 参考
  - [CentOS 8发布下载，附新功能/新特性介绍](https://ywnz.com/linuxxz/5941.html) 

> CentOS7下默认的防火墙为 `firewalld` 
```
参数解释
1、firwall-cmd：是Linux提供的操作firewall的一个工具；
2、–permanent：表示设置为持久；
3、–add-port：标识添加的端口
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



## 1.24. SELinux
- 什么是SELinux？
  > SELinux是Security Enhanced Linux的缩写，设计的目的是避免资源的利用。SELinux 是在进行进程、文件等详细权限配置时依据的一个核心模块。由于启动网络服务的也是进程，因此刚好也是能够控制网络服务能否存取系统资源的一道关卡。

- SELinux是通过MAC(Mandatory Access Control: 强制访问控制)的方式来管理进程的，它控制的 subject 是进程，object 是该进程能否读取的文件资源。