<!--
 * @Author: JohnJeep
 * @Date: 2022-04-15 20:18:46
 * @LastEditTime: 2025-03-20 18:19:46
 * @LastEditors: JohnJeep
 * @Description: Kubernetes 学习
-->
# 1. 简介

Kubernetes 最初是由Google内部项目 Borg 演化而来的，刚开始研发的时候仅仅只有 3 个人，后来得到 Google 高层的批准，将项目开源，吸引跟多的人参与进来。



# 2. 思考

国外的工程师的创造力是非常、非常的强，产生的想发都是源自于解决当前生产所面临的问题。他们产生了一个不错的想法，有平台和技术做支撑，能快速的去实现，并推广给大众，让全世界的人都能去用。（2022/4/8 9:30 Kubernetes 纪录片思考）


# 3. Core

Kubernets 管理 docker 流程

![](../figures/Kubernetes-Architecture.jpg)




## 3.1. Stateful(有状态)

- 用来控制有状态服务，StatefulSet 中的每个 Pod 的名字都是事先确定的，不能更改。
- StatefulSet 中 Pod 的名字的作用：是关联与该Pod对应的状态。
- StatefulSet 做的只是将确定的 Pod 与确定的存储关联起来保证状态的连续性。
- 每个 Pod 挂载自己独立的存储，如果一个 Pod 出现故障，从其他节点启动一个同样名字的 Pod，要挂载上原来 Pod 的存储继续以它的状态提供服务。
  	

适用场景： 

1. MySQL 、PostgreSQL数据库服务
2. ZooKeeper、etcd 集群化管理服务
3. 作为一种比普通容器更稳定可靠的模拟虚拟机的机制



## 3.2. Deployment

部署是一个比 ReplicaSet 更广的 API 对象，可以是创建一个新的服务，更新一个新的服务，也可以是滚动升级一个服务。



## 3.3. Service

- 每个 Service 对应一个集群内有效的虚拟 IP，集群内部通过虚拟IP访问服务。
- Replica Set、Replica Controller 和 Deployment 只是保证了支撑服务的微服务 Pod 的数量，但是没有解决如何访问这些服务的问题。

## 3.4. stateless(无状态)
- 所控制的 Pod 的名字是随机设置的，一个 Pod 出故障了就被丢弃掉，在另一个地方重启一个新的 Pod，名字变了。名字和启动在哪儿都不重要，重要的只是 Pod 总数。
- 一般不挂载存储或者挂载共享存储，保存的是所有 Pod 共享的状态。

### 3.4.1. DaemonSet

### 3.4.2. ReplicaSet

### 3.4.3. Replication Controller




## 3.5. Headless Service

Kubernetes 的 Headless Service 是一种特殊类型的服务，它允许直接访问 Pod 而不需要通过 Service 进行负载均衡。

通常情况下，Kubernetes 中的 Service 会为一组 Pod 提供一个虚拟的稳定的网络终点，通过负载均衡将请求分发给这组 Pod中的任意一个。但是有时候，我们可能需要直接访问每个 Pod，而不需要负载均衡。

Headless Service 可以通过设置 ClusterIP 为 "None" 来创建。创建 Headless Service 后，Kubernetes 将不会为该 Service 分配一个虚拟的 ClusterIP，而是为每个 Pod 分配一个 DNS 条目。这样，我们就可以通过 Pod 的 DNS 名称直接访问每个Pod，而不需要经过 Service 的负载均衡。

Headless Service 对于一些特定的使用场景非常有用，比如数据库集群或者分布式系统，因为这些系统通常需要直接访问每个Pod，并且不需要负载均衡。

使用场景

- 不需要负载均衡和 service IP。
- 用于服务发现机制的项目或者中间件，如 kafka 和 zookeeper 之间进行 leader 选举，采用的是实例之间的实例 IP 通讯。因为 ZK 集群的 Pod 之间需要相互识别后，进行选举状态才会变为就绪，使用无头服务完美的解决这个问题。
- 发现所有的 Pod，包括未就绪的 Pod。




# 4. References

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
