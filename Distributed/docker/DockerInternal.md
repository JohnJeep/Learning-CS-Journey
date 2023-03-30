# 理解 Docker 内部原理

1. Docke 容器本质上是宿主机的进程
2. namespace 实现了资源隔离
3. cgroups实现了资源限制
4. 写时复制机制（copy-on-write）实现了高效的文件操作。

#### namespace资源隔离

linux 内核提拱了 6 种 namespace 隔离的系统调用，如下图所示，但是真正的容器还需要处理许多其他工作。

| namespace | 系统调用参数  | 隔离内容                   |
| --------- | ------------- | -------------------------- |
| UTS       | CLONE_NEWUTS  | 主机名或域名               |
| IPC       | CLONE_NEWIPC  | 信号量、消息队列和共享内存 |
| PID       | CLONE_NEWPID  | 进程编号                   |
| Network   | CLONE_NEWNET  | 网络设备、网络战、端口等   |
| Mount     | CLONE_NEWNS   | 挂载点（文件系统）         |
| User      | CLONE_NEWUSER | 用户组和用户组             |

实际上，linux 内核实现 namespace 的主要目的，就是为了实现轻量级虚拟化技术服务。在同一个 namespace下的进程合一感知彼此的变化，而对外界的进程一无所知。这样就可以让容器中的进程产生错觉，仿佛自己置身一个独立的系统环境中，以达到隔离的目的。





# Reference

- Docker Internals: http://docker-saigon.github.io/post/Docker-Internals/



