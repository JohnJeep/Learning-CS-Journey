<!--
 * @Author: JohnJeep
 * @Date: 2022-05-20 17:20:51
 * @LastEditTime: 2025-11-20 12:11:00
 * @LastEditors: JohnJeep
 * @Description: systemd usage
-->

- [systemd](#systemd)
- [Unit](#unit)
- [配置 Uinit 地址](#配置-uinit-地址)

## systemd

Systemd（系统管理守护进程），最开始以GNU GPL协议授权开发，现在已转为使用GNU LGPL协议。字母d是daemon的缩写它取替并兼容传统的SysV init。事实上，CentOS和Debian，现在默认都是使用Systemd：

- CentOS 7开始预设并使用Systemd
- Ubuntu 15.04开始并预设使用Systemd

使用Systemd的优点：

- 按需启动进程，减少系统资源消耗
- 并行启动进程，提高系统启动速度


## Unit
Systemd引入了一个核心配置：Unit（单元配置）。事实上，Systemd管理的每个进程，都是一个Unit。相当于任务块。一个有12种模式：

Service unit：系统服务
Target unit：多个Unit构成的一个组
Device Unit：硬件设备
Mount Unit：文件系统的挂载点
Automount Unit：自动挂载点
Path Unit：文件或路径
Scope Unit：不是由 Systemd 启动的外部进程
Slice Unit：进程组
Snapshot Unit：Systemd 快照，可以切回某个快照
Socket Unit：进程间通信的 socket
Swap Unit：swap 文件
Timer Unit：定时器

## 配置 Uinit 地址
- /usr/lib/systemd/system/：推荐地址。
- /run/systemd/system/：系统执行过程中所产生的服务脚本,这些脚本的优先级比上面的高。
- /etc/systemd/system/：管理员根据主机系统的需求所建立的执行脚本，优先级比上面的高。

```
- Unit
   - Description，服务的描述
   - Documentation，文档介绍
   - After，该服务要在什么服务启动之后启动，比如Mysql需要在network和syslog启动之后再启动
- Install
   - WantedBy，值是一个或多个Target，当前Unit激活时(enable)符号链接会放入/etc/systemd/system目录下面以Target名+.wants后缀构成的子目录中
   - RequiredBy，它的值是一个或多个Target，当前Unit激活(enable)时，符号链接会放入/etc/systemd/system目录下面以Target名+.required后缀构成的子目录中
   - Alias，当前Unit可用于启动的别名
   - Also，当前Unit激活(enable)时，会被同时激活的其他Unit
- Service
   - Type，定义启动时的进程行为。它有以下几种值。
   - Type=simple，默认值，执行ExecStart指定的命令，启动主进程
   - Type=forking，以 fork 方式从父进程创建子进程，创建后父进程会立即退出
   - Type=oneshot，一次性进程，Systemd 会等当前服务退出，再继续往下执行
   - Type=dbus，当前服务通过D-Bus启动
   - Type=notify，当前服务启动完毕，会通知Systemd，再继续往下执行
   - Type=idle，若有其他任务执行完毕，当前服务才会运行
   - ExecStart，启动当前服务的命令
   - ExecStartPre，启动当前服务之前执行的命令
   - ExecStartPost，启动当前服务之后执行的命令
   - ExecReload，重启当前服务时执行的命令
   - ExecStop，停止当前服务时执行的命令
   - ExecStopPost，停止当其服务之后执行的命令
   - RestartSec，自动重启当前服务间隔的秒数
   - Restart，定义何种情况 Systemd 会自动重启当前服务，可能的值包括always（总是重启）、on-success、on-failure、on-abnormal、on-abort、on-watchdog
   - TimeoutSec，定义 Systemd 停止当前服务之前等待的秒数
   - Environment，指定环境变量

```

重载配置
```
systemctl daemon-reload
```

启动服务
```
systemctl start service_name
```

停止服务
```
systemctl stop service_name
```

systemctl设置开机自启
```
systemctl enable service_name
```

查看Unit单元是否设置了开机自启
```
systemctl is-enable service_name
```

systemctl取消开机自启
```
systemctl enable service_name
```

查看所有的单元服务
```
systemctl list-units
```

