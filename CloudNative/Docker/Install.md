# Docker 安装

官方的源很慢，使用阿里的:

unload 原的docker

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

```
sudo docker run hello-world
```



## 换源

## 更换方法

Ubuntu采用`apt`作为软件安装工具，其镜像源列表记录在`/etc/apt/source.list`文件中。

首先将`source.list`复制为`source.list.bak`备份，然后将`source.list`内容改为需要的镜像源列表即可。

修改完成后保存`source.list`文件，执行：

```
sudo apt update
```

等待更新完成即可。

## 常用国内镜像源

本节均为 Ubuntu 20.04 的镜像源列表。若为其他版本，将所有`focal`更改为其他版本代号即可。

常用的Ubuntu版本代号如下：

- Ubuntu 22.04：jammy
- Ubuntu 20.04：focal
- Ubuntu 18.04：bionic
- Ubuntu 16.04：xenial

> Ubuntu 通常采用“形容词+小动物”作为版本代号（默认的壁纸），在镜像源列表中只有第一个词。

此外，默认注释了代码源以提高速度，注释了预发布软件源（可能不稳定）。如有需要可以取消注释。

建议将所有常用镜像源保存在`/etc/apt`目录下，并命名为类似`source.list.aliyun`的形式，需要使用时直接复制替换`source.list`文件即可。

### 阿里云

```
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

```
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

```
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

```
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