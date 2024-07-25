<!--
 * @Author: JohnJeep
 * @Date: 2022-07-04 17:54:26
 * @LastEditors: JohnJeep
 * @LastEditTime: 2024-03-31 16:25:13
 * @Description: linux cgroups 用法
 * Copyright (c) 2024 by John Jeep, All Rights Reserved. 
-->

- [1. Introduce](#1-introduce)
- [2. 概念](#2-概念)
- [3. user space interfaces](#3-user-space-interfaces)
- [4. The Mounted Hierarchy](#4-the-mounted-hierarchy)
  - [4.1. 注意点](#41-注意点)
- [5. Reference](#5-reference)


# 1. Introduce

cgroup 是 control group 的缩写，cgroups 是 control groups 的缩写，是 Linux 内核提供的一种可以限制、记录、隔离进程组（process groups）所使用物理资源（如： cpu,memory,IO 等等）的机制。最初由 google 的  工程师提出，后来被整合进 Linux 内核。内核版本 2.6.24 开始引入，在 3.15 和 3.16 的内核版本中得到了加强。

# 2. 概念

- `hierarchy`：cgroups 从用户态看，提供了一种叫`cgroup`类型的文件系统(Filesystem)，这是一种虚拟的文件系统，并不真正保存文件，类似`/proc`。通过对这个文件系统的操作（读，写，创建子目录），告诉内核，你希望内核如何控制进程对资源的使用。文件系统本身是层级的，所以构成了`hierarchy`。
- `task`：进程(`process`)在cgroups中称为task，`taskid`就是`pid`。
- `subsystem`：cgroups支持的所有可配置的资源称为subsystem。例如`cpu`是一种subsystem，`memory`也是一种subsystem。linux内核在演进过程中subsystem是不断增加的。
- `libcgroup`：一个开源软件，提供了一组支持 cgroups 的应用程序和库，方便用户配置和使用cgroups。目前许多发行版都附带这个软件。

# 3. user space interfaces

- libcgroup

- cgroup-bin

- cgroup-tools

- cgroupfs-mount

# 4. The Mounted Hierarchy

资源管控器（也称为 cgroup 子系统）代表一种单一资源：如 CPU 时间或者内存。Linux kernel 提供一系列资源管控器，由 **systemd** 自动挂载。如需参考目前已挂载的资源管控器列表，请参见 `/proc/cgroups`，或使用 **lssubsys** 监控工具。

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

- blkio：对输入 ∕ 输出访问存取块设备设定权限。例如:磁盘，光盘以及usb等等。
- cpu：使用 cpu 调度程序为 cgroup 任务提供 cpu 的访问。
- cpuacct：自动生成 cgroup 中任务占用 CPU 资源的报告。
- cpuset：如果是多核心的 cpu，这个子系统会为 cgroup 中的任务分配单独的 cpu 和内存节点。
- devices：允许或禁止 cgroup 任务对设备的访问。
- freezer：暂停和恢复 cgroup 中任务。
- hugetlb：允许使用大篇幅的虚拟内存页，并且给这些内存页强制设定可用资源量。
- memory：对 cgroup 中的任务可用内存做出限制，并且自动生成任务占用内存资源报告。
- net_cls： 使用等级识别符（classid）标记网络数据包，这让 Linux 流量控制器（`tc` 指令）可以识别来自特定 cgroup 任务的数据包。
- perf_event： 允许使用 **perf** 工具来监控 cgroup。增加了对每 group 的监测跟踪的能力，即可以监测属于某个特定 group 的所有线程以及运行在特定 CPU 上的线程。



## 4.1. 注意点

- 全局的 `cgroup_mutex` 



在线业务或离线业务，都会做内核隔离（sched-idle）

开机时间

资源占用

沙箱 = 容器 + 虚机



# 5. Reference

- [Control groups, part 6: A look under the hood [LWN.net]](https://lwn.net/Articles/606925/)：Linux 周刊（LWN.net）分享的文章，非常不错。
- [Linux资源管理之cgroups简介](https://tech.meituan.com/2015/03/31/cgroups.html)：美团技术团队分享的 cgroups 用法。
