<!--
 * @Author: JohnJeep
 * @Date: 2020-04-04 09:46:51
 * @LastEditTime: 2020-11-17 22:42:28
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
  - [1.4. wc](#14-wc)
  - [1.5. top](#15-top)
  - [1.6. ps (process status)](#16-ps-process-status)
  - [1.7. 重定向](#17-重定向)
  - [1.8. 用户权限与链接](#18-用户权限与链接)
  - [1.9. find与grep](#19-find与grep)
  - [1.10. PID](#110-pid)
  - [1.11. 管道](#111-管道)
  - [1.12. 绝对路径与相对路径](#112-绝对路径与相对路径)
  - [1.13. 查看使用的端口](#113-查看使用的端口)
  - [1.14. 解压与压缩](#114-解压与压缩)
    - [1.14.1. 文件类型](#1141-文件类型)
    - [1.14.2. tar文件打包](#1142-tar文件打包)
  - [1.15. man命令](#115-man命令)
  - [1.16. strace](#116-strace)
  - [1.17. wget命令](#117-wget命令)
  - [1.18. 包管理](#118-包管理)
    - [1.18.1. 软件仓库](#1181-软件仓库)
    - [1.18.2. apt命令](#1182-apt命令)
    - [1.18.3. dpkg 命令](#1183-dpkg-命令)
  - [1.19. 防火墙](#119-防火墙)
    - [1.19.1. ubuntu下默认的防火墙](#1191-ubuntu下默认的防火墙)
    - [1.19.2. CentOS下默认的防火墙](#1192-centos下默认的防火墙)

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
  - 主要分区与扩展分区最多可以有四笔（ 硬盘的限制）
  - 扩展分区最多只能有一个（ 操作系统的限制）
  - 逻辑分区是由扩展分区持续切割出来的分区；
  - 能够被格式化后作为数据存取的分区是：主要分区与逻辑分区，扩展分区无法格式化；
  - 逻辑分区的数量依操作系统而不同，在Linux系统中 SATA 硬盘已经可以突破63个以上的分区限制；

- MBR分区的缺点
  - 操作系统无法抓取到 2.2T 以上的磁盘容量！
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
- `rm fileName`: 删除一个文本
- `mkdir `: 新建一个目录，创建多层的目录加参数 `-p` (--parents)
- `rmdir `: 移除一个目录
- `rm -rf `: 删除一个目录下的所有文件；尽量后面加上`-i`参数(interactive)，让系统在执行前确认一次。`r`参数表示递归(recursive)
- `mv  -v A B`: 文件A移到B处，并重名为B，`-v`显示系统执行的操作。
- `cp src dest`: 将文件src拷贝到当前目录下为dest文件；拷贝目录时，要加 `-r` 参数(recursive)
- `ls -al`: 查看隐藏的文件  
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
- `od(Octal Dump)`
  - 将指定文件内容以八进制、十进制、十六进制、浮点格式或 ASCII 编码字符方式显示，通常用于显示或查看文件中不能直接显示在终端的字符。`od` 命令系统默认的显示方式是八进制。
  - `-N BYTES`   --read-bytes=BYTES: 输出指定字节数
  - `-t` TYPE
  -  示例：`od -tx testfile`，以十六进制输出testfile，默认以四字节为一组（一列）显示。
- 查看文件内容
  - `more` (`-n`：可以显示行号)
  - `less` (`-N`：可以显示行号)
  - `cat`
  - `tac`
  - `nl` 查看文件时可以显示行号
  - `more`命令中输入`h`会显示帮助信息
  - 输入`b` 将显示上一屏的文件内容


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


## 1.4. wc
- 作用：print newline, word, and byte counts for each file.用来计算一个文件或者指定的多个文件中的行数，单词数和字符数。
- 选项参数
  - `-c`: 打印字节数
  - `-l`: 打印列数
  - `-w`:  打印word counts


## 1.5. top
- 查看CPU的使用率：`top`
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


## 1.6. ps (process status)
- 显示格式参数
  - `USER ` 用户名
  - `%CPU ` 进程占用的CPU百分比
  - `%MEM ` 占用内存的百分比
  - `VSZ  ` 该进程使用的虚拟內存量（KB）
  - `RSS  ` 该进程占用的固定內存量（KB）（驻留中页的数量）
  - `TTY  ` 该进程在那个终端上运行，若与终端无关，则显示? 若为pts/0等，则表示由网络连接主机进程。
  - `STAT ` 进程的状态
  - `START` 该进程被触发启动时间
  - `TIME ` 该进程实际使用CPU运行的时
  - `CMD  ` 命令的名称和参数
  - `UID  ` 用户ID、但输出的是用户名
  - `PID  ` 进程的ID
  - `PPID ` 父进程ID
  - `C    ` 进程占用CPU的百分比
  - `STIME` 进程启动到现在的时间

- STAT状态位常见的状态字符
  - `D` 无法中断的休眠状态（通常 IO 的进程）；
  - `R` 正在运行可中在队列中可过行的；
  - `S` 处于休眠状态；
  - `T` 停止或被追踪；
  - `W` 进入内存交换 （从内核2.6开始无效）；
  - `X` 死掉的进程 （基本很少见）；
  - `Z` 僵尸进程；
  - `<` 优先级高的进程
  - `N` 优先级较低的进程
  - `L` 有些页被锁进内存；
  - `s` 进程的领导者（在它之下有子进程）；
  - `l` 多线程，克隆线程（使用 CLONE_THREAD, 类似 NPTL pthreads）；
  - `+` 位于后台的进程组；

- `ps –ef|grep 程序名称`： 查看一个程序是否运行 
- `ps -Lf 端口号|wc -l `：  查看线程个数  


## 1.7. 重定向 
- `>` 重定向：`ls > test.txt` 将`ls`列出的所有文件放到test.txt中，并`覆盖原`来test.txt文件中的内容
- `命令 >> 文件名` 例如：`pwd >> text.tct`将用`pwd`生成的数据放到`test.txt`文件中，`不会覆盖原`来的文件，新加的文件保留到文本后面。


## 1.8. 用户权限与链接
- 用户权限修改
  - `chmod` 修改文件模式权限 
  - `chown` 修改文件的所属者
  - `chgrp` 修改文件的用户组
- `ln`: 建立链接命令
  -  `ln -s source  destination`    软连接
  - `ln 原文件 新文件 `       硬链接: 指向磁盘中文件的节点(inode),只有文件才能创建硬链接，目录不能创建。


## 1.9. find与grep
- find：按照文件属性查找 
  - 选项参数
    - `-name`: 例子：find /usr/src -name filename.txt
    - `-type`: 例子：find /usr/src -type f filename.txt
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
    - `-atime`: 访问时间， `+7` 七天以前，`-7` 最近七天以内访问过的
    - `-amin`: 访问时间（按照分钟）
    - `-mtime`: 修改时间（按照天数）
    - `-mmin `: 修改时间（按照分钟）
    - `ctime`: 文件属性修改时间
    - `cmin`: 文件属性修改时间（按照分钟）


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
    grep -r  "struct task {" /usr/src/ -n     // 搜索/usr/src/目录下包含 struct task { 的字符，并显示字符所在的行号
    ```


## 1.10. PID
- `pgrep -l xxxx(程序名称)`  只显示某个进程的PID
- `ps aux | grep xxx(程序名称)`  显示某个进程的全部信息，包括PID
- `ps ajx` 显示进程组ID
- `ulimit -a` 查看资源的上限大小 


## 1.11. 管道
> 定义：将一个命令的输出传送给另一个命令，作为另一个命令的输入，常用 `|` 表示。管道常与grep命令组合使用：`grep 命令1|命令2|命令3|·····|命令n`

- &和&&  |和||四者区别
  - `& `: 表示任务在后台执行，如要在后台运行redis-server,则有  redis-server &
  - `&&`: 表示前一条命令执行成功时，才执行后一条命令 ，如 echo 'hello‘ && echo 'world'    
  - `| `: 表示管道，上一条命令的输出，作为下一条命令参数，如 echo 'hello' | wc -l
  - `||`: 表示上一条命令执行失败后，才执行下一条命令，如 cat nofile || echo "failed"


## 1.12. 绝对路径与相对路径
- 绝对路径：一定由跟目录(`/`)写起。例如：`/usr/share/doc`  在shell脚本中一般使用绝对路径，防止因为不同的工作环境导致一些问题的发生。
- 相对路径：不是由根目录(`/`)写起。例如：`../man`  相对路径只是相对于当前的工作路径。



## 1.13. 查看使用的端口
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



## 1.14. 解压与压缩
### 1.14.1. 文件类型
- `gzip`: 压缩文件后缀(*.gz)
- `bzip2`: 压缩文件后缀(*.bz2)
- `xz`: 压缩文件后缀(*.xz)


### 1.14.2. tar文件打包     
- 参数
	- `-c(create)`: 打包文件
	- `-t(list)`: 察看打包文件的内容含有哪些文件名
	- `-x(extract)`: 解压打包文件 
      > 注意：`-c, -t, -x` 不可同时出现在一串命令行中。
	- `-v(verbose)`: 在压缩/解压缩的过程中，将正在处理的文件名显示出来
	- `-f(filename)`:  后面要立刻接要被处理的文件名！
	- `-C(direCtory: 目录)`: 将文件解压在特定的目录
	- `-p(小写)`:  保存原本文件的权限与属性，不包含根目录(/)。
	- `-P(大写)`：保留绝对路径，即允许备份的数据中中包含根目录。解压后的数据直接从根目录(/)开始。
	- `-z`：通过 gzip 的支持进行压缩/解压，此时文件名最好为  `*.tar.gz`
	- `-j`：通过 bzip2 的支持进行压缩/解压，此时文件名最好为 `*.tar.bz2`
	- `-J` ：通过 xz 的支持进行压缩/解压缩：此时文件名最好为 `*.tar.xz`
	   > 注意：`-z, -j, -J` 不可以同时出现在一串命令行中


- 打包压缩所有文件
  - `tar -zcvf 自己创建的文件名(xxx.tar.gz)  要打包压缩的路径(/etc/)`: 这样压缩的文件连要压缩文件的目录也一起给压缩了。

- 打包压缩所有文件不包含打包文件的路径：
  - `tar -zcvf tmp4.tar.gz -C etc/ .`    将etc下所有文件打包为tmp4.tar.gz 不包含etc包的路径。

- 打包一个目录，但不含该目录下的某些文件
  - `tar -zcvf bb.tar.gz --exclude=etc/apt etc`   将etc目录下除去apt文件的所有文件打包压缩为 bb.tar.gz，打包压缩时包含etc的路径。
	  1.  一定要注意排除目录的最后不要带"/", 否则exclude目录将不起作用
	  2.  压缩目录和排除目录都需要采用同样的格式，如都采用绝对路径或者相对路径

- 解压所有文件
  - `tar -zxf etc.tar.gz -C ~/Exercise_Linux/tmp`       将etc.tar.gz文件解压到~/Exercise_Linux/tmp目录下，不加 -C ~/Exercise_Linux/tmp 则只是解压到当前目录下。
  > 注意：指定解压的目录必须要首先存在，否则会出错，tar命令不会自动创建不存在的文件夹。

- 解压单一文件
  - `tar -zxvf etc.tar.gz etc/gdb`   将 etc.tar.gz压缩包中gdb文件夹解压 到当前目录下。



## 1.15. man命令
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

> man中的一些快捷操作：

名称 | 用法
---|---
/string |	向“下”搜寻string这个字串
?string |	向“上”搜寻string这个字串
n |	继续下一个搜寻
N |	反向查询 



## 1.16. strace
strace  - trace system calls and signals
- 监控用户进程与内核进程的交互
- 追踪进程的系统调用、信号传递、状态变化。


## 1.17. wget命令
- 参考
  - [wget命令详解](https://www.cnblogs.com/zhoul/p/9939601.html)

- 支持断点下载功能，同时支持FTP和HTTP下载方式，支持代理服务器设置
-  wget 下载单个文件下载
  - 下载的过程中会显示进度条，包含（下载完成百分比，已经下载的字节，当前下载速度，剩余下载时间）。


## 1.18. 包管理
Debian/Ubuntu采用 `dpkg` 进行软件包的管理，使用 `apt` 进行在线软件的升级。

CentOS/Red Hat/Fedora采用 `rpm` 进行软件包的管理，使用 `yum` 进行在线软件的升级。


### 1.18.1. 软件仓库
- 清华大学镜像网站：https://mirrors.tuna.tsinghua.edu.cn/cygwin/
- Windows
  > 常有文件程序自解压、选定安装组件和安装路径（个别不让选择路径）、添加注册表项等等，完成以后双击启动运行，卸载时找安装路径下的uninstall或者控制面板里卸载
- Linux或Unix
   - 软件包组织方式上，是将可执行程序、程序库、手册页等多种类型文件打包压缩提供，内容上一般分为预先编译好的二进制包和程序源码包两种；
   - 软件包管理方式上，不同开发者开发的软件，被打包集中统一存放在官方维护的软件仓库中，这个软件仓库就是一个软件源，和iOS/Android系统上的AppStore/应用市场等概念很像，Windows也开始使用“Windows Store”；除此外，第三方在遵守相关协议的前提下镜像（mirror）官方软件仓库成为镜像源，系统提供专门的软件包管理器用于从软件源下载、安装和卸载软件包。


### 1.18.2. apt命令
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


### 1.18.3. dpkg 命令
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


## 1.19. 防火墙
### 1.19.1. ubuntu下默认的防火墙
- `sudo ufw status` 查看防火墙当前状态
- `sudo ufw enable` 开启防火墙
- `sudo ufw disable` 关闭防火墙
- `sudo ufw version` 查看防火墙版本
- `sudo ufw default allow` 默认允许外部访问本机
- `sudo ufw default deny` 默认拒绝外部访问主机
- `sudo ufw allow 53` 允许外部访问53端口
- `sudo ufw deny 53` 拒绝外部访问53端口
- `sudo ufw allow from 192.168.0.1` 允许某个IP地址访问本机所有端口


### 1.19.2. CentOS下默认的防火墙
> CentOS下默认的防火墙为 `firewalld` 

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
