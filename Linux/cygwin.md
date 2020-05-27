```
 * @Author: JohnJeep
 * @Date: 2020-05-24 10:10:40
 * @LastEditTime: 2020-05-24 10:10:41
 * @LastEditors: Please set LastEditors
 * @Description: cygwin使用笔记
```
参考：
- [silaoA的博客: Cygwin学习路线](https://silaoa.github.io/2019/2019-06-16-Cygwin%E7%B3%BB%E5%88%97%EF%BC%88%E4%B9%9D%EF%BC%89%EF%BC%9ACygwin%E5%AD%A6%E4%B9%A0%E8%B7%AF%E7%BA%BF.html)
- [Cygwin的使用方法](https://blog.csdn.net/springone/article/details/676667)
- [Cygwin工具使用入门教程](https://www.linuxidc.com/Linux/2019-02/156967.htm)



- 查看当前cygwin版本：`cygcheck -c cygwin`
- 显示Windows下的进程 `ps -aW` 
- POSIX (Portable Operating System Interface) 


##### 文件概念

类型符 | 文件类型
---|---
- | 常规文件(regular)
d | 目录文件(directory)
c | 字符设备文件(character device)
l | 符号链接文件(symbolic link)
p, f | 数据传输文件(pipe, FIFO)
s | 套接文件(socket)


- 文件权限：可读（Read）、可写（Write）和可执行（eXecute）

项目 | 用户(user) | 群组(group) | 其它用户(other)
--- | --- | --- |---
权限 | 读  写  执行 | 读  写  执行| 读  写  执行
符号 | r  w  x | r  w  x | r  w  x
权值 | 4 2 1 | 4 2 1 | 4 2 1 


- 每一栏说明
  - 第1栏有10个字符，其中第1个字符描述类型，后边9个字符描述权限
  - 第2栏是硬链接数，删除文件其实是将链接数减1 ，减到0了就真正删除文件内容
  - 第3、4栏是文件属主及所在群组
  - 第5栏是文件大小，单位为字节（Byte）
  - 第6栏是最近访问（修改）时间
  - 第7栏是文件名，对于符号链接



- Shell命令分为三大类
  > - 查找某个命令属于哪类：`type -a <命令名称>`
  - 内建命令(built-in)
  - 外部命令
  - 用户定义函数（function）、别名（alias）


##### 命令
- `man` 是POSIX规定的帮助手册程序
- `info` Info手册页按照节点（node）组织，每个手册页文件是一个节点，手册页内支持链接到其他节点，如此组织犹如一张网，和网页类似 
- ` locale -a` 列出系统支持的所有语言环境
- `iconv` 将文本文件从一种字符编码转换为另一种字符编码  ` iconv -f  输入编码  -t  输出编码  输入文件 > 输出文件`
- ` cygcheck ` 包管理，查找或者显示包的信息



##### 终端换行
- 关于打印
  > 在机械打字机时代，打字机上有个“打印头（print head）”的零部件，打印时从左往右自动移动，满一行时需要手动推到最左边，这个动作叫“回车（Carriage Return）”，同时卷轴需要向上卷使纸张上移一行，打印头相对于纸张就是下移一行，这个动作叫做“移行（Line Feed）”。
  - ANSI标准规定，转义字符“\r”指代CR，“\n”指代LF，计算机系统早期广泛采用` CR+LF`指示换行。
  - UNIX系统时代存储资源很贵，仅采用1个字符“\n”指示换行，而MS-DOS出于兼容性采用“\r\n”指示换行，后来搬到了Windows上，而Mac系统则采用“\r”指示换行，Linux、Cygwin照搬了“\n”
  - “\r\n”换行的文本文件在Windows显示正常，在UNIX、Linux、Cygwin中行末多出1个“^M”，“^M”指真实的Ctrl-M组合字符；“\n”换行的文本文件在UNIX、Linux、Cygwin显示正常，在Windows中整个文件显示为一行。


- C语言换行符
  > C语言中虽然也有转义字符‘\r’、‘\n’，但并不保证与ASCII码CR、LF等价，在文本模式下，写入‘\n’由系统底层翻译成换行符，读入文本时换行符再由系统底层翻译为‘\n’。UNIX系统正是C语言写出来的，系统底层就使用LF作换行符，系统内外表示一致不需翻译；而MS-DOS、Windows系统底层，则在系统内外需要进行‘\n’与CR+LF的转换工作。


##### 两种编译构建方式
- ①原生（native）编译构建，即编译构建命令所运行（host）的系统环境和编译构建输出目标（target）的系统环境一致；
- ②交叉（cross）编译构建，上述target和host不一致，即在A系统环境构建出在B系统上运行的目标，这在嵌入式开发中尤为多见。
  > 系统环境：GNU的构建工具链中使用CPU指令集架构、厂商、系统内核的三元组合来指示系统环境


##### 软件仓库
- Windows
  > 常有文件程序自解压、选定安装组件和安装路径（个别不让选择路径）、添加注册表项等等，完成以后双击启动运行，卸载时找安装路径下的uninstall或者控制面板里卸载
- Linux或Unix
   - 软件包组织方式上，是将可执行程序、程序库、手册页等多种类型文件打包压缩提供，内容上一般分为预先编译好的二进制包和程序源码包两种；
   - 软件包管理方式上，不同开发者开发的软件，被打包集中统一存放在官方维护的软件仓库中，这个软件仓库就是一个软件源，和iOS/Android系统上的AppStore/应用市场等概念很像，Windows也开始使用“Windows Store”；除此外，第三方在遵守相关协议的前提下镜像（mirror）官方软件仓库成为镜像源，系统提供专门的软件包管理器用于从软件源下载、安装和卸载软件包。

