# Redis 安装教程

生产环境下不能连接外部网络，需用源码的方式编译 Redis 进行安装。在编译之前，先在有网的机器上，下载所需编译的依赖，然后拷贝到目标机器上执行依赖安装，最后再编译 Redis 源码执行安装。

## 1. 查看系统版本

```bash
root@VM-8-10-ubuntu:~# lsb_release -a
No LSB modules are available.
Distributor ID:	Ubuntu
Description:	Ubuntu 20.04.3 LTS
Release:	20.04
Codename:	focal
```

## 2. 下载Reids源码

可以从Redis官网的下载页面下载到最新的源码：

https://redis.io/downloads/

下载完成后我们对其进行解压

```bash
tar -xvf redis-6.2.6.tar.gz
```

## 3. 安装依赖

编译前我们需要先安装编译需要的依赖：

```bash
apt install libsystemd-dev libc6-dev
```

然后进入 deps 目录，编译剩余需要的依赖，输入以下命令以进行：

```bash
cd redis-6.2.6/deps
make hiredis linenoise hdr_histogram lua jemalloc -j16
```

## 4. 编译 Redis

编译依赖完成后就可以开始编译 Redis

```bash
# 启用对 systemd 的支持
# 将 Redis 编译成可以与 systemd 进行集成的二进制版本，方便使用 systemd 管理 Redis 服务
make USE_SYSTEMD=yes
```

如果系统为多核可以加 `-j` 线程数来开启多线程编译提升速度，如下文代码所示(此处使用16线程)：

```bash
make USE_SYSTEMD=yes -j16
```

> 启用对 systemd 的支持，需要先安装 `libsystemd-dev` 包，它提供了 systemd 的开发头文件和库。

编译完成后如有需要可以进行测试，需要安装 `tcl` 以支持测试，检查 Redis 的功能是否正常。

```bash
apt install tcl
make test
```

> `Tcl` 是一种脚本语言（**Tool Command Language**），它的库和工具在许多应用中作为嵌入式语言或测试脚本使用。在 Redis 的编译过程中，它主要用于运行 Redis 的测试脚本。
>
> Tcl 在 Redis 中 **并非用于核心功能**，而是用于 **运行测试脚本**。Redis 的测试框架依赖 Tcl 解释器来执行测试套件，因此需要在编译后运行 `make test` 时安装该依赖。

接着将其安装到系统

```bash
make install
```

## 5. **验证是否启用了 systemd 支持**

在编译完成后，运行以下命令检查 Redis 是否包含 `systemd` 支持：

```bash
redis-server --help | grep systemd
```

如果输出包含 `--with-systemd`，说明 Redis 已成功启用 systemd 支持。



## 6. 创建服务

这里我们使用 systemd 来创建服务，而不是使用过时的 `init.d`

首先复制一份redis配置到/etc下

```bash
cp redis.conf /etc/
```

打开`redis.conf`，修改**以下内容**：

修改监听IP，开启外网连接：

```bash
bind 0.0.0.0
```

找到 `supervised` 并去掉注释，将其设置为 `systemd`，要不然 `systemd` 不会检测到 `redis` 启动成功

```bash
supervised systemd
```

然后为redis设置密码：

找到 `requirepass` 并去掉注释，将"foobared"修改为你自己需要的密码，**这点很重要，没设置容易被攻击！！**

```bash
requirepass foobared
```

然后复制redis自带的示例服务文件到systemd，并且创建好redis的数据目录

```bash
cd utils
cp systemd-redis_server.service /etc/systemd/system/redis-server.service
mkdir -p /var/lib/redis
```

下面将编辑服务文件

```bash
cd /etc/systemd/system/
vim redis-server.service
```

去掉后一个`ExecStart` 的注释，将上方原有的注释掉，并将下方的配置文件目录改成我们自己的

```bash
ExecStart=/usr/local/bin/redis-server /etc/redis.conf
```

然后设置工作目录，去掉下方 `WorkingDirectory` 的注释

```bash
WorkingDirectory=/var/lib/redis
```

改好的文件看起来应该是这样子的

```ini
[Unit]
Description=Redis data structure server
Documentation=https://redis.io/documentation
#Before=your_application.service another_example_application.service
#AssertPathExists=/var/lib/redis
Wants=network-online.target
After=network-online.target

[Service]
#ExecStart=/usr/local/bin/redis-server --supervised systemd --daemonize no
## Alternatively, have redis-server load a configuration file:
ExecStart=/usr/local/bin/redis-server /etc/redis.conf
LimitNOFILE=10032
NoNewPrivileges=yes
#OOMScoreAdjust=-900
#PrivateTmp=yes
Type=notify
TimeoutStartSec=infinity
TimeoutStopSec=infinity
UMask=0077
#User=redis
#Group=redis
WorkingDirectory=/var/lib/redis

[Install]
WantedBy=multi-user.target
```

## 7. 开启服务并设置开机启动

首先看一下服务配的对不对，如果status 显示 Active 则正常。

```bash
systemctl start redis-server.service
systemctl status redis-server.service
```

正常之后直接设置开机启动即可

```bash
systemctl enable redis-server.service
```

或者一条命令搞定

```bash
systemctl enable --now redis-server.service
```

重新加载服务

```
sudo systemctl daemon-reload
```

查看是否设置为开机启动

```bash
systemctl is-enabled redis
```

查看所有已设置为开机启动的服务

```bash
systemctl list-unit-files --type=service | grep enabled
```

## 8. 非root用户运行redis

首先创建用户

```bash
useradd redis -s /sbin/nologin -b /var/lib
```

然后设置运行用户

```bash
vim /etc/systemd/system/redis-server.service

去掉下面两个注释
User=redis
Group=redis
```

设置完成之后重载systemd并且重启redis-server

```bash
systemctl daemon-reload && systemctl restart redis-server.service
```

配置完成后可以查看下是不是redis用户在跑，显示User=redis就说明配置成功

```bash
root@VM-8-10-ubuntu:/var/lib# systemctl show -pUser redis-server.service 
User=redis
```



## 9. 集群配置

### 1.9.1. 配置 Redis 节点

每个 Redis 实例都需要一个配置文件。你可以复制默认配置文件并做修改。

配置文件修改（以 `/etc/redis/redis.conf` 为例）：

- **启用集群模式**：在配置文件中启用集群模式。

  ```bash
  cluster-enabled yes
  cluster-config-file nodes.conf
  cluster-node-timeout 5000
  ```

- **绑定网络接口**：确保 Redis 实例可以被集群中的其他实例访问。修改 `bind` 配置项，或使用 `0.0.0.0` 绑定所有接口：

  ```bash
  bind 0.0.0.0
  ```

- **端口配置**：Redis 集群每个节点需要至少 2 个端口（一个是标准端口，另一个用于集群通信）。默认端口是 6379，而集群通信端口是 `6379 + 10000`。

  ```bash
  port 6379
  cluster-announce-port 6379
  cluster-announce-bus-port 16379
  ```

重复上面的步骤，修改 6 个 Redis 节点的配置文件（分别配置不同的端口）

- `redis-7000.conf`、`redis-7001.conf`、`redis-7002.conf` 等。

### 1.9.2. 启动 Redis 实例

使用不同的端口启动 Redis 实例：

```
redis-server /path/to/redis-7000.conf
redis-server /path/to/redis-7001.conf
redis-server /path/to/redis-7002.conf
redis-server /path/to/redis-7003.conf
redis-server /path/to/redis-7004.conf
redis-server /path/to/redis-7005.conf
```

### 9.3. 创建 Redis 集群

在一台机器上（或集群中的任意机器），使用 `redis-cli` 创建集群。假设你已经启动了 6 个 Redis 实例，且它们分别监听在 7000 到 7005 端口。

使用以下命令创建集群：

```
redis-cli --cluster create \
  <node1-ip>:7000 \
  <node2-ip>:7001 \
  <node3-ip>:7002 \
  <node4-ip>:7003 \
  <node5-ip>:7004 \
  <node6-ip>:7005 \
  --cluster-replicas 1
```

- `--cluster-replicas 1` 表示每个主节点有一个从节点。
- `redis-cli` 会自动为你分配槽并将数据分配到集群的节点上。

如果一切正常，`redis-cli` 会显示成功信息，集群已创建。

### 9.4. 验证集群状态

创建完集群后，可以使用 `redis-cli` 连接到集群中的任意节点，检查集群状态：

```bash
redis-cli -c -p 7000
> cluster info
```

这将显示集群的当前状态，如是否是一个正常的集群、节点数量等。

也可以使用 `cluster nodes` 查看所有节点的详细信息：

```
eredis-cli -c -p 7000
> cluster nodes
```

### 9.5. 访问集群

通过 `redis-cli` 连接到集群，执行命令时，`redis-cli` 会自动将请求路由到正确的节点。使用 `-c` 参数启用集群模式：

```
redis-cli -c -p 7000
```

然后你可以执行常规的 Redis 命令，比如：

```
set key value
get key
```

### 9.6. 维护集群

- **添加节点**：你可以通过 `redis-cli` 添加新的节点到集群。

  ```bash
  redis-cli --cluster add-node <new-node-ip>:7006 <existing-node-ip>:7000
  ```

- **删除节点**：如果你想从集群中删除某个节点：

  ```bash
  redis-cli --cluster del-node <existing-node-ip>:7000 <node-id>
  ```