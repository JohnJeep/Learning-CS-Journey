<!--
 * @Author: JohnJeep
 * @Date: 2022-04-15 20:18:46
 * @LastEditTime: 2025-04-30 02:16:29
 * @LastEditors: JohnJeep
 * @Description: Kubernetes 学习
-->

# 1. Introduce

Kubernetes 最初是由Google内部项目 Borg 演化而来的，刚开始研发的时候仅仅只有 3 个人，后来得到 Google 高层的批准，将项目开源，吸引跟多的人参与进来。



# 2. Thinking

国外的工程师的创造力是非常、非常的强，产生的想发都是源自于解决当前生产所面临的问题。他们产生了一个不错的想法，有平台和技术做支撑，能快速的去实现，并推广给大众，让全世界的人都能去用。（2022/4/8 9:30 Kubernetes 纪录片思考）


# 3. kubernets cluster architecture

Kubernetes cluster 由一个 control plane 和一组运行容器化应用程序(applications)的工作机器（称为 nodes）组成。每个 cluster 至少需要一个 worker node 才能运行 Pod。

worker nodes 托管作为应用程序工作负载组件的 Pod。 control plane 管理集群中的 worker node 和 Pod。在生产环境中， control plane 通常跨多台计算机运行，一个集群通常运行多个 nodes，从而提供容错和高可用性。

![alt text](../figures/k8s-cluster-components.png)

Kubernets 管理 docker 流程

![](../figures/Kubernetes-Architecture.jpg)


## 3.1. Control plane components

control plane 的组件对集群做出全局决策（例如，调度），以及检测和响应集群事件（例如，当 Deployment 的 replicas 字段不满足时启动新的 pod）。

Control plane 组件可以在集群中的任何计算机上运行。但是，为简单起见，设置脚本通常在同一台机器上启动所有 control plane 组件，并且不会在此机器上运行用户容器。有关跨多台计算机运行的控制平面设置示例。

- API Server:

  集群统一入口，以 Restful 方式交给 etcd 存储，暴露 Kubernetes HTTP API 的核心组件服务器。

- Scheduler

  查找尚未绑定到节点的 Pod，并将每个 Pod 分配给合适的节点。

- etcd

  为所有 API 服务器数据提供一致且高度可用的 key value store。

- Controller

  运行 controller 以实现 Kubernetes API 行为。

Controller

- 确保预期的 Pod副本数量
- 确保所有的 node 运行同一个 Pod
- 可部署一次性任务和定时任务
- 可部署 stateless, stateful



controller types
- Node controller: 负责在节点宕机时进行通知和响应。
- Job controller: 监视代表一次性任务的 Job objects，然后创建 Pod 来运行这些任务直到完成。
- EndpointSlice controller: 填充 EndpointSlice 对象（以提供 Service 和 Pod 之间的链接）。
- ServiceAccount controller: 为新命名空间创建默认 ServiceAccount。

## 3.2. Node components

Node 组件在每个节点上运行，维护正在运行的 Pods 并提供 Kubernetes runtime environment。

- kubelet

  在 cluster 中的每个 node 上运行的代理(agent)，它确保 containers 在 Pod 中运行。kubelet 采用一组通过各种机制提供的 PodSpec，并确保这些 PodSpec 中描述的 containers 正在运行且健康(running and healthy)。kubelet 不管理不是由 Kubernetes 创建的(create)container。

- kube-proxy(optional)

  kube-proxy 是一个网络代理，它在集群中的每个 node 上运行，实现了 Kubernetes Service 概念的一部分。
  
  kube-proxy 维护节点上的网络规则。这些网络规则(network rules )允许从集群内部或外部的网络 session 到 Pod 进行网络通信。
  
  kube-proxy 使用 operating system packet filtering layer，如果有并且可用 。否则，kube-proxy 会转发流量本身(forwards the traffic itself)。
  
  如果你使用一个网络插件，它自己实现 Service 的数据包转发(packet forwarding )，并提供与 kube-proxy 等效的行为，那么你不需要在集群中的 node 上运行 kube-proxy。
  
- Container runtime

  Container runtime 是 Kubernetes 能够高效的运行容器的一个基本组件。它负责管理 Kubernetes 环境中 container 的执行和生命周期(execution and lifecycle)。

  Kubernetes 支持的容器运行时有： containerd、CRI-O 以及其它实现了 Kubernetes CRI（[Container Runtime Interface](https://github.com/kubernetes/community/blob/master/contributors/devel/sig-node/container-runtime-interface.md)）的组件。



# 4. Install

## 4.1. 安装环境

Ubuntu24.04 server，kubernets 1.33 版本。

## 4.2. 主机名配置

分别在 master节点、node1、node2节点的的终端执行下面的命令，改变主机名

```bash
hostnamectl set-hostname k8s-master
hostnamectl set-hostname k8s-node1
hostnamectl set-hostname k8s-node2
```

## 4.3. IP地址配置

Ubuntu 24.04 使用 netplan 设置 IP，改变`/etc/netplan/50-cloud-init.yaml` 中内容，设置如下：

```bash
network:
  version: 2
  ethernets:
    ens33:
      dhcp4: no
      addresses: [192.168.218.11/24]
      routes:
        - to: default
          via: 192.168.218.2
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
```

改完后，执行 `netplan apply` 是配置文件生效。

## 4.4. 主机名与IP地址解析

`/etc/hosts` 文件末尾添加每台机器的IP和host，以便集群之间主机名能互相解析

```bash
192.168.218.10 k8s-master
192.168.218.11 k8s-node1
192.168.218.12 k8s-node2
```

## 4.5. 时间同步配置

查看时间

```bash
# date
Wed Apr 30 00:02:54 CST 2025
```

操作系统默认是美国时区，要改为中国的时区

```
// 更换时区
timedatectl set-timezone Asia/Shanghai
```

安装 ntpdate 命令

```bash
apt install ntpdate 
```

使用 ntpdate 命令同步时间

```
ntpdate time1.aliyun.com
```

通过计划任务实现时间同步

明天凌晨0点同步一次

```
# crontab -e 
0 0 * * * ntpdate time1.aliyum.com
```

重启生效

```
systemctl restart cron
```



## 4.6. 设置 `C.UTF-8` 作为系统默认区域（locale）(可选)

查看系统是否已安装 `C.UTF-8`

```
locale -a
```

手动编辑 `/etc/default/locale`

```
sudo vim /etc/default/locale
```

修改或添加以下内容

```
LANG=C.UTF-8
LC_ALL=C.UTF-8
```

重新加载环境变量或重启系统

```
source /etc/default/locale
```

检查当前 locale

```
# locale
LANG=C.UTF-8
LANGUAGE=
LC_CTYPE="C.UTF-8"
LC_NUMERIC="C.UTF-8"
LC_TIME="C.UTF-8"
LC_COLLATE="C.UTF-8"
LC_MONETARY="C.UTF-8"
LC_MESSAGES="C.UTF-8"
LC_PAPER="C.UTF-8"
LC_NAME="C.UTF-8"
LC_ADDRESS="C.UTF-8"
LC_TELEPHONE="C.UTF-8"
LC_MEASUREMENT="C.UTF-8"
LC_IDENTIFICATION="C.UTF-8"
LC_ALL=
```



## 4.7. 配置内核转发及网桥过滤

创建加载内核模块文件

```
cat << EOF | tee /etc/modules-load.d/k8s.conf
overlay
br_netfilter
EOF
```

手动加载模块

```
modprobe overlay
modprobe br_netfilter
```

查看已加载模块

```bash
#lsmod | grep overlay
overlay               212992  0

# lsmod | grep br_netfilter
br_netfilter           32768  0
bridge                421888  1 br_netfilter
```

添加网桥过滤及内核转发配置文件

```
cat << EOF | tee /etc/sysctl.d/k8s.conf
net.bridge.bridge-nf-call-ip6tables = 1
net.bridge.bridge-nf-call-iptables = 1
net.ipv4.ip_forward = 1
EOF
```

加载内核参数

```
sysctl --system
```



## 4.8. 安装ipset和ipvsadm

安装ipset和ipvsadm

```
apt install ipset ipvsadm
```

配置 ipvsadm 模块

```
cat << EOF | tee /etc/modules-load.d/ipvs.conf
ip_vs
ip_vs_rr
ip_vs_wrr
ip_vs_sh
nf-conntrack
EOF
```

创建加载模块脚本文件

```
cat << EOF | tee ipvs.sh
#!/bin/sh

modprobe -- ip_vs
modprobe -- ip_vs_rr
modprobe -- ip_vs_wrr
modprobe -- ip_vs_sh
modprobe -- nf-conntrack
EOF
```

执行脚本文件，加载模块

```
sh ipvs.sh
```

## 4.9. 关闭SWAP分区

`swapoff -a` 是临时关闭swap分区，重启后swap还在。

永远关闭 `swap` 分区，需重启操作系统。

查看当前 Swap

```
swapon --show
```

输出示例：

```
NAME      TYPE      SIZE USED PRIO
/dev/sda5 partition 2G   0B   -2
```

**注释掉 `/etc/fstab` 中的 Swap 项**

```
sudo vim /etc/fstab
```

找到类似以下的行（以 `swap` 或 `none` 开头）：

```
# 如果是 Swap 分区
/dev/sda5 none swap sw 0 0

# 如果是 Swap 文件
/swapfile none swap sw 0 0
```

在行首添加 `#` 注释掉：

```
# /dev/sda5 none swap sw 0 0
# /swapfile none swap sw 0 0
```

**删除 Swap 文件（可选）**

如果 Swap 是文件（如 `/swapfile`），可以彻底删除：

```
sudo rm /swapfile
```

重启系统，验证是否生效。

```
free -h 

swapon --show # 如果输出中 Swap 行显示 0，说明已成功禁用。
```

---

二进制方式搭建集群步骤

1. 创建多台虚拟机，安装Linux操作系统
2. 操作系统初始化
3. 给 etcd, apiserver 安装自签证书
4. 部署 etcd 集群
5. 部署 master 组件
   1. kube-apiserver
   2. kube-controller-manager
   3. kub-scheduler
   4. etcd
6. 部署 node 组件
   1. kubetlet
   2. kube-proxy
   3. docker
   4. etcd
7. 部署集群网络

简单部署：直接使用kubeadm + cert-manager 来实现。

## 4.10. kubernets集群

**安装指定版本**

```
apt install -y kubelet=1.33.0  kubeadm=1.33.0  kubectl=1.33.0
```

kubelet、kubeadm、kubectl 安装后锁点版本，防止自动更新

```
apt-mark hold kubelet kubeadm kubectl
```

解锁版本，执行更新

```
apt-mark unhold kubelet kubeadm kubectl
```



### 4.10.1. 配置 kubelet

为了实现容器运行时使用的 `cgroupdriver` 与 `kubelet` 使用的 cgroup 的一致性，建议修改下面的内容

```
# vim /etc/default/kubelet
KUBELET_EXTAR_ARGS="--cgroup-driver=systemd"
```

设置 kubelet 开机自启

```
systemctl enable kubelet
```

### 4.10.2. 集群初始化

查看 kubeadm 版本

```bash
root@k8s-master02:~# kubeadm version
kubeadm version: &version.Info{Major:"1", Minor:"33", EmulationMajor:"", EmulationMinor:"", MinCompatibilityMajor:"", MinCompatibilityMinor:"", GitVersion:"v1.33.0", GitCommit:"60a317eadfcb839692a68eab88b2096f4d708f4f", GitTreeState:"clean", BuildDate:"2025-04-23T13:05:48Z", GoVersion:"go1.24.2", Compiler:"gc", Platform:"linux/amd64"}

```

**生成部署配置文件**

```
kubeadm config print init-defaults > kubeadm-config.yaml
```


# 5. Workloads

## 5.1. Pod

- 最小的 Deployment 单元。
- 一组 container 的集合。
- 同一个 Pod 内部共享网络命名空间(namespace)、共享同一个 Linux 协议栈。
- 短暂的生命周期，重启后资源被销毁。



## 5.2. Workload Management

### 5.2.1. Deployment

Deployment 是一个比 ReplicaSet 更广的 API 对象，可以create、update、rolling-update一个新的服务。



### 5.2.2. ReplicaSet

ReplicaSet 是可以独立使用的，但还是建议使用 Deployment 来自动管理ReplicaSet，这样就无需担心跟其他机制不兼容的问题（比如：ReplicaSet 不支持 rolling-update，但Deployment 支持）。

**注**：**ReplicaSet 和 Deployment 是为无状态服务而设计的。**



### 5.2.3. ReplicationController

新版 k8s 中建议用 ReplicaSet 来取代 ReplicationController。跟 ReplicaSet 没有本质的区别，只是名字不一样。



### 5.2.4. StatefulSet

StatefulSe 用来解决有状态服务的问题。StatefulSet 中的每个 Pod 的名字都是事先确定的，不能更改。

适用场景： 

- 稳定的持久化存储。Pod 重新调度后还是能访问到相同的持久化数据，基于PVC 实现。
- 稳定的网络标志。Pod 重新调度后 PodName 和 HostName 不变，基于 Headless service 来实现。
- 有序存储，有序扩展。Pod 是有顺序的，在部署或者扩展的时候要依据定义的顺序依次执行，基于 Init containers 来实现。
- 有序收缩，有序删除。



### 5.2.5. DaemonSet

DaemonSet 确保全部或者一些 Node 上运行有且只有一个 Pod 的副本。当有 Node 加入 Cluster 时，会为他们新增一个 Pod ，当有 Node 从 Cluster 移除时，这些 Pod 会被回收，删除 DaemonSet 将删除它创建的所有 Pod。

使用场景：

- 运行集群存储 daemon。例如：在每个 Node 上运行 glusterd
- 在每个 Node 上运行日志收集 daemon。例如：logstash，fluentd
- 在每个 Node 上运行监控 daemon。例如：Prometheus Node exporter

### 5.2.6. Job

Job 负责批处理任务。即执行一次的任务，它保证批处理任务的一个或多个 Pod 成功结束。

### 5.2.7. Cron Job

Cron Job 是基于时间的 Job。即在给定的时间点只运行一次，周期性的在给定时间点运行。





# 6. Service

> service：定义一组 Pod 访问的规则。

- 每个 Service 对应一个集群内有效的虚拟 IP，集群内部通过虚拟IP访问服务。
- Replica Set、Replica Controller 和 Deployment 只是保证了支撑服务的微服务 Pod 的数量，但是没有解决如何访问这些服务的问题。

常见分类

- Cluster IP: 默认类型，自动分配一个仅 Cluster 内部可以访问的虚拟 IP。

- Node Port: 在 Cluster IP 的基础上，为 service 在每台机器上绑定一个端口，这样就可以通过 `NodeIP:NodePort` 这样的方式来访问 service。 

- LoadBalancer: 在 NodePort 的基础上，借助 cloud provider创建一个外部负载均衡器，并将请求转发到 `NodeIP:NodePort`

  借助云服务商来实现的，云服务商需要收费。

- Extern Name: 把集群外部的服务引入到集群内部来，在集群内部直接使用，没有任何代理被创建。

实现方式

- userspace
- iptables
- ipvs

SVC(service) 机制



### 6.0.1. Headless Service

Kubernetes 的 Headless Service 是一种特殊类型的服务，它允许直接访问 Pod 而不需要通过 Service 进行负载均衡。**即没有 Cluster IP 的 service。**

通常情况下，Kubernetes 中的 Service 会为一组 Pod 提供一个虚拟的稳定的网络终点，通过负载均衡将请求分发给这组 Pod中的任意一个。但是有时候，我们可能需要直接访问每个 Pod，而不需要负载均衡。

Headless Service 可以通过设置 ClusterIP 为 "None" 来创建。创建 Headless Service 后，Kubernetes 将不会为该 Service 分配一个虚拟的 ClusterIP，而是为每个 Pod 分配一个 DNS 条目。这样，我们就可以通过 Pod 的 DNS 名称直接访问每个Pod，而不需要经过 Service 的负载均衡。

Headless Service 对于一些特定的使用场景非常有用，比如数据库集群或者分布式系统，因为这些系统通常需要直接访问每个Pod，并且不需要负载均衡。

使用场景

- 不需要负载均衡和 service IP。
- 用于服务发现机制的项目或者中间件，如 kafka 和 zookeeper 之间进行 leader 选举，采用的是实例之间的实例 IP 通讯。因为 ZK 集群的 Pod 之间需要相互识别后，进行选举状态才会变为就绪，使用无头服务完美的解决这个问题。
- 发现所有的 Pod，包括未就绪的 Pod。

### 6.0.2. Stateless

- 所控制的 Pod 的名字是随机设置的，一个 Pod 出故障了就被丢弃掉，在另一个地方重启一个新的 Pod，名字变了。名字和启动在哪儿都不重要，重要的只是 Pod 总数。
- 一般不挂载存储或者挂载共享存储，保存的是所有 Pod 共享的状态。

# 7. Network

k8s 中有 3 层网络

![alt text](../figures/k8s-network.png)

- Node Network

  是一个真是的网络。

- Pod Network

  是一个内部的虚拟网络。

- Service Network

  是一个内部的虚拟网络。





# 8. Storage

## 8.1. Volume

## 8.2. secret

## 8.3. PV



# 9. Schedule



# 10. Security

认证

鉴权

访问控制

原理及其流程



# 11. Helm

**Helm** 是一个流行的开源工具，用于简化 Kubernetes 应用的**打包、部署和管理**。它被称作“Kubernetes 的包管理器”（类似于 Linux 中的 `apt` 或 `yum`），通过抽象化复杂的 Kubernetes YAML 文件，提供更高效的应用生命周期管理。

## 11.1. Helm 的核心概念

1. **Chart**
   - Helm 的打包格式，是一个预定义的目录结构，包含运行 Kubernetes 应用所需的所有资源（Deployment、Service、ConfigMap 等）。
   - 类似于软件安装包（如 `.deb`/`.rpm`），但专为 Kubernetes 设计。
2. **Release**
   - 当 Chart 被部署到 Kubernetes 集群时，会生成一个唯一的 **Release**（即一个运行中的实例）。
   - 同一 Chart 可多次部署，每次生成独立的 Release（例如：开发、测试、生产环境）。
3. **Repository（Repo）**
   - 存储和共享 Chart 的仓库，可以是公共的（如 [Artifact Hub](https://artifacthub.io/)）或私有的。
4. **Values.yaml**
   - 通过动态配置参数（如镜像版本、资源限制）实现 Chart 的定制化，避免修改原始模板。

------

## 11.2. Helm 的核心优势

1. **简化复杂应用部署**
   - 将多个 Kubernetes 资源（Deployment、Service、Ingress 等）打包成一个 Chart，一键部署。
2. **支持版本化和回滚**
   - 记录每次 Release 的版本历史，可快速回滚到之前的稳定版本。
3. **参数化配置**
   - 通过 `values.yaml` 动态注入配置，适应不同环境（如开发/生产）。
4. **共享与复用**
   - 社区提供大量现成 Chart（如 MySQL、Nginx、Prometheus），可直接使用或二次开发。

------

## 11.3. Helm 基本命令示例

```bash
# 添加公共仓库
helm repo add bitnami https://charts.bitnami.com/bitnami

# 搜索 Chart
helm search repo bitnami/nginx

# 安装 Chart（生成 Release）
helm install my-nginx bitnami/nginx -f values.yaml

# 查看已部署的 Release
helm list

# 升级 Release
helm upgrade my-nginx bitnami/nginx --set replicaCount=2

# 回滚到上一版本
helm rollback my-nginx 1

# 卸载 Release
helm uninstall my-nginx
```

------

## 11.4. Helm 版本演进

- **Helm 2**：早期版本，依赖服务端组件 `Tiller`（存在安全风险，已淘汰）。
- **Helm 3**（当前主流）：移除 Tiller，提升安全性，支持 CRD 和更完善的依赖管理。

------

## 11.5. 适用场景

- 部署有状态/无状态应用（如数据库、Web 服务）。
- 管理微服务依赖（如同时部署前端、后端和中间件）。
- 实现 GitOps 流程（配合 ArgoCD 等工具）。



# 12. Addons(插件)



# 13. References

- 官方英文文档: https://kubernetes.io
- 官方中文文档: https://kubernetes.io/zh/docs/home
- Making the Kubernetes Service Abstraction Scale using eBPF, [译] 利用 eBPF 支撑大规模 K8s Service (LPC, 2019)：https://linuxplumbersconf.org/event/4/contributions/458
- 基于 BPF/XDP 实现 K8s Service 负载均衡 (LPC, 2020)https://linuxplumbersconf.org/event/7/contributions/674
- 深入理解 Kubernetes 网络模型：自己实现 Kube-Proxy 的功能: https://cloudnative.to/blog/k8s-node-proxy
- Containerd 使用教程：https://icloudnative.io/posts/getting-started-with-containerd
- Kubernetes 的层级命名空间介绍：https://icloudnative.io/posts/introducing-hierarchical-namespaces
- Kubernetes 的设计理念：https://jimmysong.io/kubernetes-handbook/concepts/concepts.html
- Kubernetes 部署教程：https://zhuanlan.zhihu.com/p/641521752
- Docker Containers and Kubernetes: An Architectural Perspective: https://dzone.com/articles/docker-containers-and-kubernetes-an-architectural
- Kubernetes 简明教程: https://lailin.xyz/post/k8s-tutorials.html

