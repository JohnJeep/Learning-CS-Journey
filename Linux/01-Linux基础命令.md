### 基本命令
- `touch 文件名 ` 新建一个文本
- `rm 文件名 ` 删除一个文本
- `mkdir ` 新建一个目录
- `rmdir ` 移除一个目录
- ` rm -rf ` 删除一个目录下的所有文件；尽量后面加上`-i`参数(interactive)，让系统在执行前确认一次。`r`参数表示递归(recursive)
- ` mv  -v A B` 文件A移到B处，并重名为B，`-v`显示系统执行的操作。
- ` find / -name filename.txt`  查找/目录下的`filename.txt` 文件  
- `whereis ls`  查看所有包含`ls`命令的位置  
- `ln`(建立链接命令)
  - 软连接    `ln -s 原文件 新文件`
  - 硬链接    `ln 原文件 新文件 `
- `ps –ef|grep tomcat` 查看一个程序是否运行 
- ` ls -al` 查看隐藏的文件  
- `chmod` 修改权限 
- ` ps -Lf 端口号|wc -l `  查看线程个数   
- `>` 重定向：`ls > test.txt` 将`ls`列出的所有文件放到`test.txt`中
- `命令 >> 文件名` 例如：`pwd >> text.tct`将用`pwd`生成的数据放到`test.txt`文件的 最后
- `bg` 进程转到后台
- `fg` 进程转到前台



### 用户名操作
- 增加一个用户 ` useradd`
- 删除用户 ` userdel+用户名 `

### 管道

    定义：将一个命令的输出传送给另一个命令，作为另一个命令的输入

- 用法：命令1|命令2|命令3|·····|命令n
- grep（查文件中包含的一个相关命令)
格式：        `grep "要搜索的内容" xxx.txt `
   - grep  -n
  - grep  -v 
  -  开头` ^  `       结尾` $ `


### 查看使用的端口
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
  -  获取所有侦听 TCP 端口的列表 `lsof -nP -iTCP -sTCP:LISTEN `


### 常用 Bash终端快捷键
参考：[Bash 快捷键大全](https://linux.cn/article-5660-1.html)

    Ctrl开头的快捷键一般是针对字符的，而Alt开头的快捷键一般是针对词的。

1. 控制命令
- `Ctrl + d` : 退出当前终端（end of file）
- `Ctrl + c` : 强制终止当前命令
- `Ctrl + z` : 将当前任务暂停并挂在后台
- `Ctrl + s`: 冻结当前terminal的stdin，键盘输入的数据不会马上在terminal显示，而是恢复冻结后，在当前光标之前显示输入的数据。（Suspend）
 `Ctrl +  q`: 恢复当前terminal的stdin（Resume）

2. 大小写
- `esc + u:`  将当前光标之后以空格隔开的单词或者字符转换为大写，包括当前光标(upper)
- `esc + l:`  将当前光标之后以空格隔开的单词或者字符转换为大小，包括当前光标(lower)
- `esc + c:`  将当前光标之后以空格隔开的单词首字母转换为大写

3. 编辑
>擦除：输入的数据还在缓冲内存中，没有被删除，可以复制
- `Ctrl + u` : 擦除当前光标之前的字符，不包括当前光标
- `Ctrl + k` : 擦除从当前光标之后到行尾的所有字符，包括当前光标
- `Ctrl + y`: 从当前光标处粘贴之前擦除的字符
- `Ctrl + w` : 删除当前光标之前以空格隔开的任意长度的字符，不包括当前光标
- `Ctrl + t`: 当前光标处字符与前一个字符交换位置
- `Ctrl + t`: 撤销之前擦除的所有字符
- `Alt + d`: 擦除当前光标之后的一个单词，包括当前光标
- `Alt + r`:取消当前所有改变的操作，若当前命令从历史记录中来，返回上一次历史记录的原始状态，若当前命令是手动输入，则会清空当前行
- `Alt + t`: 当前光标处单词与前一个单词交换位置

4. 历史记录
- `Ctrl + r`: 搜索历史记录
- `Ctrl + g`: 退出当前搜索模式（在搜索历史记录下）
- `fc - l`: 默认从最后往前显示 16 条历史记录。



### wget命令
参考:
[wget命令详解](https://www.cnblogs.com/zhoul/p/9939601.html)

- 支持断点下载功能，同时支持FTP和HTTP下载方式，支持代理服务器设置
-  wget 下载单个文件下载：
   下载的过程中会显示进度条，包含（下载完成百分比，已经下载的字节，当前下载速度，剩余下载时间）。



### 文件压缩与解压
-  仅打包，并非压缩
    - tar -xvf FileName.tar         # 解包
    - tar -cvf FileName.tar DirName # 将DirName和其下所有文件（夹）打包


- .gz
    - gunzip FileName.gz      # 解压1
    - gzip -d FileName.gz       # 解压2
    - gzip FileName                  # 压缩，只能压缩文件


-  .tar.gz 和 .tgz
    - tar -zxvf FileName.tar.gz                                      # 解压
    - tar -zcvf FileName.tar.gz DirName                  # 将DirName和其下所有文件（夹）压缩
    - tar -C DesDirName -zxvf FileName.tar.gz     # 解压到目标路径


-  感觉.zip占用空间比.tar.gz大
    - unzip -d /home/user/ FileName.zip	# 解压到指定文件目录/home/user
    - zip FileName.zip DirName    		# 将DirName本身压缩
    - zip -r FileName.zip DirName 		# 压缩，递归处理，将指定目录下的所有文件和子目录一并压缩
    - unzip -n -d /temp test.zip              # 解压时，不想覆盖已经存在的文件，加上-n参数
    - unzip -l test.zip			# 只查看zip压缩包中包含哪些文件，不进行解压缩
    - unzip -t test.zip			# 检查zip文件是否损坏


-  mac和linux并没有自带rar，需要去下载
   - rar x FileName.rar      # 解压
   - rar a FileName.rar DirName # 压缩


### apt命令
- apt-cache search # ------(package 搜索包)
- apt-cache show #------(package 获取包的相关信息，如说明、大小、版本等)
- apt-get install # ------(package 安装包)
- apt-get install # -----(package --reinstall 重新安装包)
- apt-get -f install # -----(强制安装, "-f = --fix-missing"当是修复安装吧...)
- apt-get remove #-----(package 删除包)
- apt-get remove --purge # ------(package 删除包，包括删除配置文件等)
- apt-get autoremove --purge # ----(package 删除包及其依赖的软件包+配置文件等
- apt-get update #------更新源
- apt-get upgrade #------更新已安装的包
- apt-get dist-upgrade # ---------升级系统
- apt-get dselect-upgrade #------使用 dselect 升级
- apt-cache depends #-------(package 了解使用依赖)
- apt-cache rdepends # ------(package 了解某个具体的依赖,当是查看该包被哪些包依赖吧...)
- apt-get build-dep # ------(package 安装相关的编译环境)
- apt-get source #------(package 下载该包的源代码)
- apt-get clean && apt-get autoclean # --------清理下载文件的存档 && 只清理过时的包
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



### dpkg 命令
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

