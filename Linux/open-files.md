# 1. 为什么要修改可打开的文件描述符的数量

进程每打开一个文件（linux下一切皆文件，包括socket），都会消耗一定的内存资源。如果有不怀好心的人启动一个进程来无限的创建和打开新的文件，会让服务器崩溃。所以linux系统出于安全角度的考虑，在多个位置都限制了可打开的文件描述符的数量，包括系统级、用户级、进程级。这三个限制的含义和修改方式如下：

- 系统级：当前系统可打开的最大数量，通过 `fs.file-max` 参数可修改
- 用户级：指定用户可打开的最大数量，修改 `/etc/security/limits.conf`
- 进程级：单个进程可打开的最大数量，通过 `fs.nr_open` 参数可修改



## 1.1. 系统级别修改

## 1.2. 用户级别修改

## 1.3. 进程级别修改





查看系统参数

```sh
[root@KF-CFT-AP2 ~]# ulimit -a
core file size          (blocks, -c) 0          # 设定core文件的最大值，单位为 block。
data seg size           (kbytes, -d) unlimited
scheduling priority             (-e) 0
file size               (blocks, -f) unlimited
pending signals                 (-i) 62795
max locked memory       (kbytes, -l) 64
max memory size         (kbytes, -m) unlimited
open files                      (-n) 1024      # 一个进程可以打开文件描述符的数量的最大值
pipe size            (512 bytes, -p) 8
POSIX message queues     (bytes, -q) 819200
real-time priority              (-r) 0
stack size              (kbytes, -s) 10240
cpu time               (seconds, -t) unlimited
max user processes              (-u) 62795
virtual memory          (kbytes, -v) unlimited
file locks                      (-x) unlimited
```



## 1.4. root 用户修改后其他用户不生效

在root用户修改为65536后，用其他用户登录服务器检测`ulimit -n` 还是1024。那么就是该用户未生效。

修改 `/etc/ssh/sshd_config` 中配置，将 UsePAM 项值设置为 yes，表示使用 PAM 模块来加载。 

```
# UsePAM no
UsePAM yes
```

修改完后，重启服务

```sh
service sshd restart
```





系统参数修改临时有效

```sh
直接在 shell 中修改参数值，比如
1. 修改 core 文件大小
	[root@KF-CFT-AP2 ~]# ulimit -c unlimited
2. 修改文件打开的个数 open files
	[root@KF-CFT-AP2 ~]# ulimit -n 2000

其它的参数同理修改
```

系统参数修永久有效

```sh
修改 /etc/security/limits.conf 文件，直接在文件后面追加自己要添加的内容，完成后，登出当前用户，然后再登录，查看改变的值。

[root@KF-CFT-AP2 limits.d]# vim /etc/security/limits.conf
#        - core - limits the core file size (KB)
#        - data - max data size (KB)
#        - fsize - maximum filesize (KB)
#        - memlock - max locked-in-memory address space (KB)
#        - nofile - max number of open files
#        - rss - max resident set size (KB)
#        - stack - max stack size (KB)
#        - cpu - max CPU time (MIN)
#        - nproc - max number of processes
#        - as - address space limit (KB)
#        - maxlogins - max number of logins for this user
#        - maxsyslogins - max number of logins on the system
#        - priority - the priority to run user process with
#        - locks - max number of file locks the user can hold
#        - sigpending - max number of pending signals
#        - msgqueue - max memory used by POSIX message queues (bytes)
#        - nice - max nice priority allowed to raise to values: [-20, 19]
#        - rtprio - max realtime priority
#
#<domain>      <type>  <item>         <value>
#

#*               soft    core            0
#*               hard    rss             10000
#@student        hard    nproc           20
#@faculty        soft    nproc           20
#@faculty        hard    nproc           50
#ftp             hard    nproc           0
#@student        -       maxlogins       4

# End of file

# 修改打开文件数
* hard nofile 3000
* soft nofile 3000
```

*：表示所有的用户。





注意：有时修改了 `/etc/security/limits.conf` 文件并没有达到自己所预期的内容，感觉是没有生效。可能的原因：

1. 加载了系统中 `/etc/profile` 文件中对系统参数的修改。优先级第二高
2. 加载了系统中 `/etc/security/limits.d/` 目录下文件的修改，比如：`90-nproc.conf` 或者 `20-nproc.conf`文件。优先级第三高

hard limit 只是作为 soft limit 的上限，soft limit 才是你设置的系统当前限制。当你设置 hard limit 后，soft limit 的值就只能小于 hard limit 。普通用户可以降低 hard limit 的值，但是不能提高它，只有 root 用户才能提高 hard limit。



-------------------

于nproc配置信息的扩展说明:

对`max user processes`的配置, Linux系统默认先读取`/etc/security/limits.conf` 中的信息, 如果`/etc/security/limits.d/`目录下还有配置文件的话, 也会依次遍历读取, 最终, `/etc/security/limits.d/`中的配置会覆盖`/etc/security/limits.conf` 中的配置.

另外, `max open files`和`max user processes`是不能配置`unlimited`的 —— 极不安全的设置, 此时系统会使用默认的配置值. 对`nproc`而言, 默认值的计算方法为:

```sh
# 查看系统的 max user processes
[kf@ZHCS-AP1 ~]$ cat /proc/sys/kernel/threads-max
128108

# 计算公式为: 
default_nproc = max_threads / 2;
# 其中, max_threads = mempages / (8 * THREAD_SIZE / PAGE_SIZE);
# mempages是机器的物理页面个数, THREAD_SIZE=8K, 所以, 计算公式为: 
default_nproc = max_threads / 2 
              = (mempages * PAGE_SIZE) / ( 2 * 8 *THREAD_SIZE ) 
              = total_memory / 128K;
              
# 计算本机默认nproc配置: 
cat /proc/meminfo | grep MemTotal
MemTotal:       115571480 kB

echo "115571480 / 128" | bc
902902

ulimit -u
902682
# 算出来default_nproc = 902902, 和实际的902682很接近, 
# 因为物理页面会存储一些关键数据, 所以实际的比计算出来的要小一些.
```

------------------

用户登录的时候执行sh脚本的顺序： 
    /etc/profile.d/file 
    /etc/profile 
    /etc/bashrc 
    /mingjie/.bashrc 
    /mingjie/.bash_profile 

    由于ulimit -n的脚本命令加载在第二部分，用户登录时由于权限原因在第二步还不能完成ulimit的修改，所以ulimit的值还是系统默认的1024。




参考：

- [/etc/security/limits.conf 详解与配置](https://www.cnblogs.com/operationhome/p/11966041.html)
- [Linux-PAM 官方文档](http://www.linux-pam.org/)
- [Linux下PAM模块学习总结](https://www.cnblogs.com/kevingrace/p/8671964.html)
- [Linux下设置最大文件打开数nofile及nr_open、file-max](https://www.cnblogs.com/zengkefu/p/5635153.html)
- [Linux - 修改系统的max open files、max user processes ](https://www.cnblogs.com/shoufeng/p/10620480.html)
- [一台Linux服务器最多能支撑多少个TCP连接？](https://mp.weixin.qq.com/s/QuCxkSjdM_E12lXlpnhKIQ) 从底层原理去解释。

