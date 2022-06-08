# Introduce

cgroup 是 control group 的缩写，Cgroups 是 control groups 的缩写，是 Linux 内核提供的一种可以限制、记录、隔离进程组（process groups）所使用物理资源（如： cpu,memory,IO 等等）的机制。最初由 google 的  工程师提出，后来被整合进 Linux 内核。内核版本 2.6.24 开始引入，在 3.15 和 3.16 的内核版本中得到了加强。

# user space interfaces

- libcgroup

- cgroup-bin

- cgroup-tools

- cgroupfs-mount

# The Mounted Hierarchy

```bash
[root@CentOS7 cgroup]# pwd
/sys/fs/cgroup
[root@CentOS7 cgroup]# ls -al
total 0
drwxr-xr-x. 13 root root 340 May 26 21:00 .
drwxr-xr-x.  8 root root   0 May 26 21:00 ..
drwxr-xr-x.  2 root root   0 May 26 21:00 blkio
lrwxrwxrwx.  1 root root  11 May 26 21:00 cpu -> cpu,cpuacct
lrwxrwxrwx.  1 root root  11 May 26 21:00 cpuacct -> cpu,cpuacct
drwxr-xr-x.  2 root root   0 May 26 21:00 cpu,cpuacct
drwxr-xr-x.  2 root root   0 May 26 21:00 cpuset
drwxr-xr-x.  5 root root   0 May 26 21:00 devices
drwxr-xr-x.  2 root root   0 May 26 21:00 freezer
drwxr-xr-x.  2 root root   0 May 26 21:00 hugetlb
drwxr-xr-x.  2 root root   0 May 26 21:00 memory
lrwxrwxrwx.  1 root root  16 May 26 21:00 net_cls -> net_cls,net_prio
drwxr-xr-x.  2 root root   0 May 26 21:00 net_cls,net_prio
lrwxrwxrwx.  1 root root  16 May 26 21:00 net_prio -> net_cls,net_prio
drwxr-xr-x.  2 root root   0 May 26 21:00 perf_event
drwxr-xr-x.  5 root root   0 May 26 21:00 pids
drwxr-xr-x.  5 root root   0 May 26 21:00 systemd
```

# Reference

- [Control groups, part 6: A look under the hood [LWN.net]](https://lwn.net/Articles/606925/)：Linux 周刊（LWN.net）分享的文章，非常不错。
- [Linux资源管理之cgroups简介](https://tech.meituan.com/2015/03/31/cgroups.html)：美团技术团队分享的 cgroups 用法。
