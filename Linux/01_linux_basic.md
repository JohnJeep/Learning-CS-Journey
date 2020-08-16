<!--
 * @Author: JohnJeep
 * @Date: 2020-04-04 09:46:51
 * @LastEditTime: 2020-08-15 19:11:13
 * @LastEditors: Please set LastEditors
 * @Description: Linux基础用法笔记
--> 

<!-- TOC -->

- [1. Linux Basic](#1-linux-basic)
  - [1.1. 基本命令](#11-基本命令)
  - [1.2. 查看文件内容](#12-查看文件内容)
  - [1.3. 重定向](#13-重定向)
  - [1.4. 用户权限与链接](#14-用户权限与链接)
  - [1.5. find与grep](#15-find与grep)
  - [1.6. PID](#16-pid)
  - [1.7. 管道](#17-管道)
  - [1.8. 查看使用的端口](#18-查看使用的端口)
  - [1.9. 常用 Bash终端快捷键](#19-常用-bash终端快捷键)
  - [1.10. man命令](#110-man命令)
  - [1.11. strace](#111-strace)
  - [1.12. wget命令](#112-wget命令)
  - [1.13. apt命令](#113-apt命令)
  - [1.14. dpkg 命令](#114-dpkg-命令)

<!-- /TOC -->

# 1. Linux Basic

## 1.1. 基本命令
- `touch 文件名 ` 文件不存在新建一个文本；文件存在时，修改文件创建的时间。
- `rm 文件名 ` 删除一个文本
- `mkdir ` 新建一个目录，创建多层的目录加参数 `-p` (--parents)
- `rmdir ` 移除一个目录
- ` rm -rf ` 删除一个目录下的所有文件；尽量后面加上`-i`参数(interactive)，让系统在执行前确认一次。`r`参数表示递归(recursive)
- `mv  -v A B` 文件A移到B处，并重名为B，`-v`显示系统执行的操作。
- `cp src dest` 将文件src拷贝到当前目录下为dest文件；拷贝目录时，要加 `-r` 参数(recursive)
- `ps –ef|grep tomcat` 查看一个程序是否运行 
- `ls -al` 查看隐藏的文件  
- `ps -Lf 端口号|wc -l `  查看线程个数  


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


- 查看CPU的使用率：`top`命令
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



## 1.2. 查看文件内容
- `more` (`-n`：可以显示行号)
- `less` (`-N`：可以显示行号)
- `cat`
- `tac`
- `nl` 查看文件时可以显示行号
- `more`命令中输入`h`会显示帮助信息
- 输入`b` 将显示上一屏的文件内容


## 1.3. 重定向 
- `>` 重定向：`ls > test.txt` 将`ls`列出的所有文件放到test.txt中，并`覆盖原`来test.txt文件中的内容
- `命令 >> 文件名` 例如：`pwd >> text.tct`将用`pwd`生成的数据放到`test.txt`文件中，`不会覆盖原`来的文件，新加的文件保留到文本后面。



## 1.4. 用户权限与链接
- 用户权限修改
  - `chmod` 修改文件模式权限 
  - `chown` 修改文件的所属者
  - `chgrp` 修改文件的用户组

- `ln`(建立链接命令)
  -  `ln -s 原文件  新文件`    软连接
  - `ln 原文件 新文件 `       硬链接: 指向磁盘中文件的节点(inode),只有文件才能创建硬链接，目录不能创建。


## 1.5. find与grep
- find：按照文件属性查找 
  - 按文件名 ` find 查找目录 -name filename.txt`  查找指定目录下的`filename.txt` 文件 
  - 按文件大小 `find 查找目录 -size 条件(+10k)` 查找指定目录下大于10k的文件
  - 按文件类型 `find 查找目录 -type 类型`   类型包括：f(file), d(directory), l(link), c(char), d(device), s(socket), b(block)


- `grep` 按文件内容查找
  - `grep -r 查找内容 查找路径 ` 


## 1.6. PID
- `pgrep -l xxxx(程序名称)`  只显示某个进程的PID
- `ps aux | grep xxx(程序名称)`  显示某个进程的全部信息，包括PID
- `ps ajx` 显示进程组ID
- `ulimit -a` 查看资源的上限大小 


## 1.7. 管道
定义：将一个命令的输出传送给另一个命令，作为另一个命令的输入
- 用法：命令1|命令2|命令3|·····|命令n
- grep（查文件中包含的一个相关命令)
格式：`grep "要搜索的内容" xxx.txt `
  - grep  -n
  - grep  -v 
  -  开头` ^  `       
  -  结尾` $ `


## 1.8. 查看使用的端口
- 侦听端口：应用程序或进程侦听的网络端口，充当通信端点
- 同一个 IP 地址上不能用两个不同的服务去侦听同一端口
- ` netstat` 检查端口
  - ` -t `  显示 TCP 端口。
  - ` -u `  显示 UDP 端口。
  - ` -n `  显示数字地址而不是主机名。
  - ` -l `  仅显示侦听端口。
  - ` -p `  显示进程的 PID 和名称
    - ` netstat -tunlp ` 列出正在侦听的所有 TCP 或   UDP 端口
    - Proto - 套接字使用的协议。
    - Local Address - 进程侦听的 IP 地址和端口号。
    - PID/Program name  - PID 和进程名称
- ` ss ` 检查端口
  - `  ss -tunlp`
- ` lsof ` 检查端口
  - `lsof -nP -iTCP -sTCP:LISTEN ` 获取所有侦听 TCP 端口的列表 



## 1.9. 常用 Bash终端快捷键
- 参考
  - [Bash 快捷键大全](https://linux.cn/article-5660-1.html)


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
- `Ctrl + t` 撤销之前擦除的所有字符
- `Alt + d`  擦除当前光标之后的一个单词，包括当前光标
- `Alt + r`  取消当前所有改变的操作，若当前命令从历史记录中来，返回上一次历史记录的原始状态，若当前命令是手动输入，则会清空当前行
- `Alt + t` 当前光标处单词与前一个单词交换位置
- `Ctrl + h` 删除当前光标前面的字符，每次删除一个字符
- `Ctrl + d` 删除当前光标后面的字符，每次删除一个字符
- `Ctrl + b` 光标向后(backward)移动，与 `←` 等价
- `Ctrl + f` 光标向前移动(forward)，与 `→` 等价


4. 历史记录
- `Ctrl + r`: 搜索历史记录
- `Ctrl + g`: 退出当前搜索模式（在搜索历史记录下）
- `fc - l`: 默认从最后往前显示 16 条历史记录。
- `Ctrl + p` 向上查找历史命令，与 `↑` 等价
- `Ctrl + n` 向下查找历史命令，与 `↓` 等价


5. 其它命令
- `cd - ` 在两个相邻的目录之间进行切换。
- ` ; ` 在一条行中执行多条命令，采用 ; 实现
- ` && ` 仅在上一个命令成功的情况下，才能执行后面的多个命令
- `lsblk` 以树状的格式列出块设备



## 1.10. man命令
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


## 1.11. strace
strace  - trace system calls and signals
- 监控用户进程与内核进程的交互
- 追踪进程的系统调用、信号传递、状态变化。



## 1.12. wget命令
- 参考
  - [wget命令详解](https://www.cnblogs.com/zhoul/p/9939601.html)


- 支持断点下载功能，同时支持FTP和HTTP下载方式，支持代理服务器设置
-  wget 下载单个文件下载
  - 下载的过程中会显示进度条，包含（下载完成百分比，已经下载的字节，当前下载速度，剩余下载时间）。



## 1.13. apt命令
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



## 1.14. dpkg 命令
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

