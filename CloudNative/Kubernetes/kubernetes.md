<!--
 * @Author: JohnJeep
 * @Date: 2022-04-15 20:18:46
 * @LastEditTime: 2025-04-15 14:43:32
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

# 4. Workloads

## 4.1. Pod

- 最小的 Deployment 单元。
- 一组 container 的集合。
- 同一个 Pod 内部共享网络命名空间(namespace)、共享同一个 Linux 协议栈。
- 短暂的生命周期，重启后资源被销毁。



## 4.2. Workload Management

### 4.2.1. Deployment

Deployment 是一个比 ReplicaSet 更广的 API 对象，可以create、update、rolling-update一个新的服务。



### 4.2.2. ReplicaSet

ReplicaSet 是可以独立使用的，但还是建议使用 Deployment 来自动管理ReplicaSet，这样就无需担心跟其他机制不兼容的问题（比如：ReplicaSet 不支持 rolling-update，但Deployment 支持）。

**注**：**ReplicaSet 和 Deployment 是为无状态服务而设计的。**



### 4.2.3. ReplicationController

新版 k8s 中建议用 ReplicaSet 来取代 ReplicationController。跟 ReplicaSet 没有本质的区别，只是名字不一样。



### 4.2.4. StatefulSet

StatefulSe 用来解决有状态服务的问题。StatefulSet 中的每个 Pod 的名字都是事先确定的，不能更改。

适用场景： 

- 稳定的持久化存储。Pod 重新调度后还是能访问到相同的持久化数据，基于PVC 实现。
- 稳定的网络标志。Pod 重新调度后 PodName 和 HostName 不变，基于 Headless service 来实现。
- 有序存储，有序扩展。Pod 是有顺序的，在部署或者扩展的时候要依据定义的顺序依次执行，基于 Init containers 来实现。
- 有序收缩，有序删除。



### 4.2.5. DaemonSet

DaemonSet 确保全部或者一些 Node 上运行有且只有一个 Pod 的副本。当有 Node 加入 Cluster 时，会为他们新增一个 Pod ，当有 Node 从 Cluster 移除时，这些 Pod 会被回收，删除 DaemonSet 将删除它创建的所有 Pod。

使用场景：

- 运行集群存储 daemon。例如：在每个 Node 上运行 glusterd
- 在每个 Node 上运行日志收集 daemon。例如：logstash，fluentd
- 在每个 Node 上运行监控 daemon。例如：Prometheus Node exporter

### 4.2.6. Job

Job 负责批处理任务。即执行一次的任务，它保证批处理任务的一个或多个 Pod 成功结束。

### 4.2.7. Cron Job

Cron Job 是基于时间的 Job。即在给定的时间点只运行一次，周期性的在给定时间点运行。





# 5. Service

> service：定义一组 Pod 访问的规则。

- 每个 Service 对应一个集群内有效的虚拟 IP，集群内部通过虚拟IP访问服务。
- Replica Set、Replica Controller 和 Deployment 只是保证了支撑服务的微服务 Pod 的数量，但是没有解决如何访问这些服务的问题。

常见分类

- Cluster IP: 默认类型，自动分配一个仅 Cluster 内部可以访问的虚拟 IP。

- Node Port: 在 Cluster IP 的基础上，为 service 在每台机器上绑定一个端口，这样就可以通过 `NodeIP:NodePort` 这样的方式来访问 service。 

- LoadBalancer: 在 NodePort 的基础上，借助 cloud provider创建一个外部负载均衡器，并将请求转发到 `NodeIP:NodePort`

  借助云服务商来实现的，云服务商需要收费。

- Extern Name: 把集群外部的服务引入到集群内部来，在集群内部直接使用，没有任何代理被创建。

实现方式

- userspace
- iptables
- ipvs

SVC(service) 机制




### 5.0.1. Headless Service

Kubernetes 的 Headless Service 是一种特殊类型的服务，它允许直接访问 Pod 而不需要通过 Service 进行负载均衡。**即没有 Cluster IP 的 service。**

通常情况下，Kubernetes 中的 Service 会为一组 Pod 提供一个虚拟的稳定的网络终点，通过负载均衡将请求分发给这组 Pod中的任意一个。但是有时候，我们可能需要直接访问每个 Pod，而不需要负载均衡。

Headless Service 可以通过设置 ClusterIP 为 "None" 来创建。创建 Headless Service 后，Kubernetes 将不会为该 Service 分配一个虚拟的 ClusterIP，而是为每个 Pod 分配一个 DNS 条目。这样，我们就可以通过 Pod 的 DNS 名称直接访问每个Pod，而不需要经过 Service 的负载均衡。

Headless Service 对于一些特定的使用场景非常有用，比如数据库集群或者分布式系统，因为这些系统通常需要直接访问每个Pod，并且不需要负载均衡。

使用场景

- 不需要负载均衡和 service IP。
- 用于服务发现机制的项目或者中间件，如 kafka 和 zookeeper 之间进行 leader 选举，采用的是实例之间的实例 IP 通讯。因为 ZK 集群的 Pod 之间需要相互识别后，进行选举状态才会变为就绪，使用无头服务完美的解决这个问题。
- 发现所有的 Pod，包括未就绪的 Pod。

### 5.0.2. Stateless

- 所控制的 Pod 的名字是随机设置的，一个 Pod 出故障了就被丢弃掉，在另一个地方重启一个新的 Pod，名字变了。名字和启动在哪儿都不重要，重要的只是 Pod 总数。
- 一般不挂载存储或者挂载共享存储，保存的是所有 Pod 共享的状态。

# 6. Network

k8s 中有 3 层网络

![alt text](../figures/k8s-network.png)

- Node Network

  是一个真是的网络。

- Pod Network

  是一个内部的虚拟网络。

- Service Network

  是一个内部的虚拟网络。





# 7. Storage

## 7.1. Volume

## 7.2. secret

## 7.3. PV



# 8. Schedule



# 9. Security

认证

鉴权

访问控制

原理及其流程



# 10. Helm

集群安装包命令



# 11. Addons(插件)



# 12. References

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

