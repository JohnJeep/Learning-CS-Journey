<!--
 * @Author: JohnJeep
 * @Date: 2023-10-12 10:38:01
 * @LastEditors: JohnJeep
 * @LastEditTime: 2024-01-06 13:58:32
 * @Description: Docker 安装教程
 * Copyright (c) 2024 by John Jeep, All Rights Reserved. 
-->

# Linux 换源

## 更换方法

Ubuntu采用`apt`作为软件安装工具，其镜像源列表记录在`/etc/apt/source.list`文件中。
首先将`source.list`复制为`source.list.bak`备份，然后将`source.list`内容改为需要的镜像源列表即可。
修改完成后保存`source.list`文件，执行：

```sh
sudo apt update
```

等待更新完成即可。

## 常用国内镜像源

本节均为 Ubuntu 20.04 的镜像源列表。若为其他版本，将所有`focal`更改为其他版本代号即可。

常用的Ubuntu版本代号如下：

```sh
Ubuntu 22.04：jammy
Ubuntu 20.04：focal
Ubuntu 18.04：bionic
Ubuntu 16.04：xenial
```

Ubuntu 通常采用“形容词+小动物”作为版本代号（默认的壁纸），在镜像源列表中只有第一个词。
此外，默认注释了代码源以提高速度，注释了预发布软件源（可能不稳定）。如有需要可以取消注释。
建议将所有常用镜像源保存在`/etc/apt`目录下，并命名为类似`source.list.aliyun`的形式，需要使用时直接复制替换`source.list`文件即可。

### 阿里云

```sh
deb http://mirrors.aliyun.com/ubuntu/ focal main restricted universe multiverse
deb http://mirrors.aliyun.com/ubuntu/ focal-security main restricted universe multiverse
deb http://mirrors.aliyun.com/ubuntu/ focal-updates main restricted universe multiverse
deb http://mirrors.aliyun.com/ubuntu/ focal-backports main restricted universe multiverse

# deb-src http://mirrors.aliyun.com/ubuntu/ focal main restricted universe multiverse
# deb-src http://mirrors.aliyun.com/ubuntu/ focal-security main restricted universe multiverse
# deb-src http://mirrors.aliyun.com/ubuntu/ focal-updates main restricted universe multiverse
# deb-src http://mirrors.aliyun.com/ubuntu/ focal-backports main restricted universe multiverse

## Pre-released source, not recommended.
# deb http://mirrors.aliyun.com/ubuntu/ focal-proposed main restricted universe multiverse
# deb-src http://mirrors.aliyun.com/ubuntu/ focal-proposed main restricted universe multiverse
```

### 清华

```sh
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal-updates main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal-backports main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal-security main restricted universe multiverse

# deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal main restricted universe multiverse
# deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal-updates main restricted universe multiverse
# deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal-backports main restricted universe multiverse
# deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal-security main restricted universe multiverse

## Pre-released source, not recommended.
# deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal-proposed main restricted universe multiverse
# deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal-proposed main restricted universe multiverse
```

### 中科大

```sh
deb https://mirrors.ustc.edu.cn/ubuntu/ focal main restricted universe multiverse
deb https://mirrors.ustc.edu.cn/ubuntu/ focal-updates main restricted universe multiverse
deb https://mirrors.ustc.edu.cn/ubuntu/ focal-backports main restricted universe multiverse
deb https://mirrors.ustc.edu.cn/ubuntu/ focal-security main restricted universe multiverse

# deb-src https://mirrors.ustc.edu.cn/ubuntu/ focal main restricted universe multiverse
# deb-src https://mirrors.ustc.edu.cn/ubuntu/ focal-updates main restricted universe multiverse
# deb-src https://mirrors.ustc.edu.cn/ubuntu/ focal-backports main restricted universe multiverse
# deb-src https://mirrors.ustc.edu.cn/ubuntu/ focal-security main restricted universe multiverse

## Pre-released source, not recommended.
# deb https://mirrors.ustc.edu.cn/ubuntu/ focal-proposed main restricted universe multiverse
# deb-src https://mirrors.ustc.edu.cn/ubuntu/ focal-proposed main restricted universe multiverse
```

### 网易163

```sh
deb http://mirrors.163.com/ubuntu/ focal main restricted universe multiverse
deb http://mirrors.163.com/ubuntu/ focal-security main restricted universe multiverse
deb http://mirrors.163.com/ubuntu/ focal-updates main restricted universe multiverse
deb http://mirrors.163.com/ubuntu/ focal-backports main restricted universe multiverse

# deb-src http://mirrors.163.com/ubuntu/ focal main restricted universe multiverse
# deb-src http://mirrors.163.com/ubuntu/ focal-security main restricted universe multiverse
# deb-src http://mirrors.163.com/ubuntu/ focal-updates main restricted universe multiverse
# deb-src http://mirrors.163.com/ubuntu/ focal-backports main restricted universe multiverse

## Pre-released source, not recommended.
# deb http://mirrors.163.com/ubuntu/ focal-proposed main restricted universe multiverse
# deb-src http://mirrors.163.com/ubuntu/ focal-proposed main restricted universe multiverse
```





# 5. Docker 安装

平台支持（Supported platforms）：Docker 引擎（Docker Engine ）支持 Linux 、macOS、Win10（通过 Docker 桌面版）等不同的平台安装，还支持[静态二进制文件](https://docs.docker.com/engine/install/binaries/)的安装。

## 5.1. CenOS7 下安装

Docker 并非一个通用的容器工具，它依赖于已存在并运行的 Linux 内核环境。Docker 实际上是在已运行的 Linux 下制造了一个隔离的文件环境，因此它执行的效率几乎等同于所部署的 Linux 主机。Docker 必须部署在带有Linux 内核的系统上。

安装前提条件

目前，CentOS 仅支持发行版中内核，要求系统为 64 位，32 位的操作系统暂时不支持，Linux 系统的内核版本为 3.8 以上。

查看 Linux 内核版本

```sh
// 查看 Linux 发行版
[root@redis_181 ~]# cat /etc/redhat-release
CentOS Linux release 7.9.2009 (Core)

// 查看内核版本、硬件架构、主机名称、操作系统类型等信息
[root@redis_181 ~]# uname -a    
Linux redis_181 3.10.0-1160.49.1.el7.x86_64 #1 SMP Tue Nov 30 15:51:32 UTC 2021 x86_64 x86_64 x86_64 GNU/Linux
```



## 5.2. Windows 下安装

Docker 官网介绍 Windows下安装 Docker有两种方式，一种是在 Windows 子系统（WSL）中安装 Docker，另一种在带有 Windows 虚拟化技术的 Windows 容器中安装。

### 5.2.1. WSL 下安装

WSL 已安装好后，在WSL2 里面的终端按照下面的步骤执行：

1. 配置 docker 源

```sh
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -

sudo add-apt-repository \
   "deb [arch=amd64] https://mirrors.tuna.tsinghua.edu.cn/docker-ce/linux/ubuntu \
   $(lsb_release -cs) \
   stable"

sudo apt update
```

2. 安装 docker-ce

   ```
   sudo apt install -y docker-ce
   ```

3. 启动 docker

   ```
   sudo service docker start
   ```

## Ubuntu 下安装

移除已安装的 docker
```
sudo apt-get remove docker docker-engine docker-ce docker.io

sudo apt-get update
```

添加docker的使用的公钥

```shell
# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://mirrors.aliyun.com/docker-ce/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

# Add the repository to Apt sources:
echo \
  "deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/docker.gpg] https://mirrors.aliyun.com/docker-ce/ubuntu \
  "$(. /etc/os-release && echo "$VERSION_CODENAME")" stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
```

安装docker-ce
```
sudo apt-get install -y docker-ce
```

启动docker
```
sudo systemctl status docker
```

运行hello-world
```sh
sudo docker run hello-world
```



# References

- https://docs.docker.com/engine/install/ubuntu/
- Control Docker with systemd: https://docs.docker.com/config/daemon/systemd/
- Install Docker Engine from binaries: https://docs.docker.com/engine/install/binaries/
- cgroupfs-mount: https://github.com/tianon/cgroupfs-mount