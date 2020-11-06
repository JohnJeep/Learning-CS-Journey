<!--
 * @Author: JohnJeep
 * @Date: 2020-04-04 09:46:51
 * @LastEditTime: 2020-11-06 12:20:45
 * @LastEditors: Please set LastEditors
 * @Description: Linux基础用法笔记
--> 

<!-- TOC -->

- [1. Linux Basic](#1-linux-basic)
  - [1.1. 基础命令](#11-基础命令)
  - [1.2. wc](#12-wc)
  - [1.3. top](#13-top)
  - [1.4. ps (process status)](#14-ps-process-status)
  - [1.5. 查看文件内容](#15-查看文件内容)
  - [1.6. 重定向](#16-重定向)
  - [1.7. 用户权限与链接](#17-用户权限与链接)
  - [1.8. find与grep](#18-find与grep)
  - [1.9. PID](#19-pid)
  - [1.10. 管道](#110-管道)
  - [1.11. 查看使用的端口](#111-查看使用的端口)
  - [1.12. 常用 Bash终端快捷键](#112-常用-bash终端快捷键)
  - [1.13. man命令](#113-man命令)
  - [1.14. strace](#114-strace)
  - [1.15. wget命令](#115-wget命令)
  - [1.16. 包管理](#116-包管理)
    - [1.16.1. apt命令](#1161-apt命令)
    - [1.16.2. dpkg 命令](#1162-dpkg-命令)
  - [1.17. 防火墙](#117-防火墙)
    - [1.17.1. ubuntu下默认的防火墙](#1171-ubuntu下默认的防火墙)
    - [1.17.2. CentOS下默认的防火墙](#1172-centos下默认的防火墙)

<!-- /TOC -->

# 1. Linux Basic
- 参考
  - [Github上Linux工具快速教程](https://github.com/me115/linuxtools_rst) ：这本书专注于Linux工具的最常用用法，以便读者能以最快时间掌握，并在工作中应用


## 1.1. 基础命令
- `touch 文件名 ` 文件不存在新建一个文本；文件存在时，修改文件创建的时间。
- `rm 文件名 ` 删除一个文本
- `mkdir ` 新建一个目录，创建多层的目录加参数 `-p` (--parents)
- `rmdir ` 移除一个目录
- `rm -rf ` 删除一个目录下的所有文件；尽量后面加上`-i`参数(interactive)，让系统在执行前确认一次。`r`参数表示递归(recursive)
- `mv  -v A B` 文件A移到B处，并重名为B，`-v`显示系统执行的操作。
- `cp src dest` 将文件src拷贝到当前目录下为dest文件；拷贝目录时，要加 `-r` 参数(recursive)
- `ls -al` 查看隐藏的文件  
- `which` 通过环境变量PATH到该路径内寻找可执行文件，用于查找可执行文
- `whereis` 查找系统中包含可以找到的所有文件
- `eject` 将光盘驱动器中的光盘轻轻弹出和收回
- `mount 设备名字 挂在目录`
  - `mount` 设备装载常用命令：`mount –t type dev dir`
  - `–t type` 是需要挂载的文件系统类型，光盘文件系统类型是：iso9660；
  - `dev` 是挂载文件系统的设备名称，光盘驱动器的设备名称是/dev/cdrom; 
  - `dir`表示挂载点，即挂载到的文件目录路径
  - 例如：`mount -t iso9660 /dev/cdrom /media/drom`
- `umout` 设备装载常用命令
   - 例如：`umount dir device […]`
- `od` 查看二进制文件信息
- `du` 查看文件使用空间大小 `-h` (human)以人类容易看懂的方式显示
- `df` 查看磁盘情况 
- `fdisk -l` 查看硬盘的情况
- `uname -a` 查看Linux版本
- `lscpu` 查看系统CPU情况
- `nslookup 域名` 查看域名对应的IP地址


## 1.2. wc
- 作用：print newline, word, and byte counts for each file.用来计算一个文件或者指定的多个文件中的行数，单词数和字符数。
- 选项参数
  - `-c`: 打印字节数
  - `-l`: 打印列数
  - `-w`:  打印word counts


## 1.3. top
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


## 1.4. ps (process status)
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
  

## 1.5. 查看文件内容
- `more` (`-n`：可以显示行号)
- `less` (`-N`：可以显示行号)
- `cat`
- `tac`
- `nl` 查看文件时可以显示行号
- `more`命令中输入`h`会显示帮助信息
- 输入`b` 将显示上一屏的文件内容


## 1.6. 重定向 
- `>` 重定向：`ls > test.txt` 将`ls`列出的所有文件放到test.txt中，并`覆盖原`来test.txt文件中的内容
- `命令 >> 文件名` 例如：`pwd >> text.tct`将用`pwd`生成的数据放到`test.txt`文件中，`不会覆盖原`来的文件，新加的文件保留到文本后面。


## 1.7. 用户权限与链接
- 用户权限修改
  - `chmod` 修改文件模式权限 
  - `chown` 修改文件的所属者
  - `chgrp` 修改文件的用户组
- `ln`(建立链接命令)
  -  `ln -s 原文件  新文件`    软连接
  - `ln 原文件 新文件 `       硬链接: 指向磁盘中文件的节点(inode),只有文件才能创建硬链接，目录不能创建。


## 1.8. find与grep
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


## 1.9. PID
- `pgrep -l xxxx(程序名称)`  只显示某个进程的PID
- `ps aux | grep xxx(程序名称)`  显示某个进程的全部信息，包括PID
- `ps ajx` 显示进程组ID
- `ulimit -a` 查看资源的上限大小 


## 1.10. 管道
> 定义：将一个命令的输出传送给另一个命令，作为另一个命令的输入，常用 `|` 表示。
- 与grep命令组合：`grep 命令1|命令2|命令3|·····|命令n`


## 1.11. 查看使用的端口
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


## 1.12. 常用 Bash终端快捷键
- 参考
  - [Bash快捷键大全](https://linux.cn/article-5660-1.html)
  - [Terminator The robot future of terminals](https://gnometerminator.blogspot.com/p/introduction.html)


> Ctrl开头的快捷键一般是针对字符的，而Alt开头的快捷键一般是针对词的。

1. 控制命令
- `Ctrl + d` : 退出当前终端（end of file）
- `Ctrl + c` : 强制终止当前命令，终止的是前台进程
- `Ctrl + z` : 将当前任务暂停并挂在后台
- `Ctrl + s`: 冻结当前terminal的stdin，键盘输入的数据不会马上在terminal显示，而是恢复冻结后，在当前光标之前显示输入的数据。（Suspend）
- `Ctrl +  q`: 恢复当前terminal的stdin（Resume）
- ` stop + job号` 将前台的进程挂起
- ` fg ` 将后台中的命令调至前台继续运行
  > `fg %jobnumber`将选中的命令调出，`%jobnumber`是通过jobs命令查到的后台正在执行的命令的序号(不是pid)
- `bg` 进程转到后台
- 终止后台进程
  - `kill + job号`
  - `kill + PID`

2. 大小写
- `esc + u:`  将当前光标之后以空格隔开的单词或者字符转换为大写，包括当前光标(upper)
- `esc + l:`  将当前光标之后以空格隔开的单词或者字符转换为大小，包括当前光标(lower)
- `esc + c:`  将当前光标之后以空格隔开的单词首字母转换为大写

3. 编辑
>擦除：输入的数据还在缓冲内存中，没有被删除，可以复制
- `Ctrl + u` 擦除当前光标之前的字符，不包括当前光标
- `Ctrl + k` 擦除从当前光标之后到行尾的所有字符，包括当前光标
- `Ctrl + y` 从当前光标处粘贴之前擦除的字符
- `Ctrl + w` 删除当前光标之前以空格隔开的任意长度的字符，不包括当前光标
- `Ctrl + t` 当前光标处字符与前一个字符交换位置
- `Ctrl + /` 撤销之前擦除的所有字符
- `Alt + d`  擦除当前光标之后的一个单词，包括当前光标
- `Alt + r`  取消当前所有改变的操作，若当前命令从历史记录中来，返回上一次历史记录的原始状态，若当前命令是手动输入，则会清空当前行
- `Alt + t`  当前光标处单词与前一个单词交换位置
- `Ctrl + h` 删除当前光标前面的字符，每次删除一个字符
- `Ctrl + d` 删除当前光标后面的字符，每次删除一个字符
- `Ctrl + b` 光标向后(backward)移动，与 `←` 等价
- `Ctrl + f` 光标向前移动(forward)，与 `→` 等价

4. 历史记录
- `Ctrl + r` 搜索历史记录
- `Ctrl + g` 退出当前搜索模式（在搜索历史记录下）
- `fc - l`   默认从最后往前显示 16 条历史记录。
- `Ctrl + p` 向上查找历史命令，与 `↑` 等价
- `Ctrl + n` 向下查找历史命令，与 `↓` 等价

5. 其它命令
- `cd - ` 在两个相邻的目录之间进行切换。
- ` ; ` 在一条行中执行多条命令，采用 ; 实现
- ` && ` 仅在上一个命令成功的情况下，才能执行后面的多个命令
- `lsblk` 以树状的格式列出块设备


## 1.13. man命令
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


## 1.14. strace
strace  - trace system calls and signals
- 监控用户进程与内核进程的交互
- 追踪进程的系统调用、信号传递、状态变化。


## 1.15. wget命令
- 参考
  - [wget命令详解](https://www.cnblogs.com/zhoul/p/9939601.html)

- 支持断点下载功能，同时支持FTP和HTTP下载方式，支持代理服务器设置
-  wget 下载单个文件下载
  - 下载的过程中会显示进度条，包含（下载完成百分比，已经下载的字节，当前下载速度，剩余下载时间）。


## 1.16. 包管理
Debian/Ubuntu采用 `dpkg` 进行软件包的管理，使用 `apt` 进行在线软件的升级。

CentOS/Red Hat/Fedora采用 `rpm` 进行软件包的管理，使用 `yum` 进行在线软件的升级。


### 1.16.1. apt命令
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
- dpkg -S filename -----查找filename属于哪个软件包
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


### 1.16.2. dpkg 命令
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


## 1.17. 防火墙
### 1.17.1. ubuntu下默认的防火墙
- `sudo ufw status` 查看防火墙当前状态
- `sudo ufw enable` 开启防火墙
- `sudo ufw disable` 关闭防火墙
- `sudo ufw version` 查看防火墙版本
- `sudo ufw default allow` 默认允许外部访问本机
- `sudo ufw default deny` 默认拒绝外部访问主机
- `sudo ufw allow 53` 允许外部访问53端口
- `sudo ufw deny 53` 拒绝外部访问53端口
- `sudo ufw allow from 192.168.0.1` 允许某个IP地址访问本机所有端口


### 1.17.2. CentOS下默认的防火墙
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
