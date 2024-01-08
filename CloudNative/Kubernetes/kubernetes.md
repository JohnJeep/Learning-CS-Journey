<!--
 * @Author: your name
 * @Date: 2022-04-15 20:18:46
 * @LastEditTime: 2022-04-26 23:51:06
 * @LastEditors: JohnJeep
 * @Description: Kubernetes 学习
 * @FilePath: 
-->
# 1. 简介

Kubernetes 最初是由Google内部项目 Borg 演化而来的，刚开始研发的时候仅仅只有 3 个人，后来得到 Google 高层的批准，将项目开源，吸引跟多的人参与进来。



# 2. 思考

国外的工程师的创造力是非常、非常的强，产生的想发都是源自于解决当前生产所面临的问题。他们产生了一个不错的想法，有平台和技术做支撑，能快速的去实现，并推广给大众，让全世界的人都能去用。（2022/4/8 9:30 Kubernetes 纪录片思考）



Kubernets 管理 docker 流程

![](../figures/Kubernetes-Architecture.jpg)

参考：[Docker Containers and Kubernetes: An Architectural Perspective](https://dzone.com/articles/docker-containers-and-kubernetes-an-architectural)



### Headless Service

Kubernetes的Headless Service是一种特殊类型的服务，它允许直接访问Pod而不需要通过Service进行负载均衡。

通常情况下，Kubernetes中的Service会为一组Pod提供一个虚拟的稳定的网络终点，通过负载均衡将请求分发给这组Pod中的任意一个。但是有时候，我们可能需要直接访问每个Pod，而不需要负载均衡。

Headless Service可以通过设置ClusterIP为"None"来创建。创建Headless Service后，Kubernetes将不会为该Service分配一个虚拟的ClusterIP，而是为每个Pod分配一个DNS条目。这样，我们就可以通过Pod的DNS名称直接访问每个Pod，而不需要经过Service的负载均衡。

Headless Service对于一些特定的使用场景非常有用，比如数据库集群或者分布式系统，因为这些系统通常需要直接访问每个Pod，并且不需要负载均衡。

使用场景

- 不需要负载均衡和 service IP。
- 用于服务发现机制的项目或者中间件，如kafka和zookeeper之间进行leader选举，采用的是实例之间的实例IP通讯。因为ZK集群的Pod之间需要相互识别后，进行选举状态才会变为就绪，使用无头服务完美的解决这个问题。
- 发现所有的 Pod，包括未就绪的Pod。



# 3. References

- 官方英文文档: https://kubernetes.io/
- 官方中文文档: https://kubernetes.io/zh/docs/home/
- Making the Kubernetes Service Abstraction Scale using eBPF, [译] 利用 eBPF 支撑大规模 K8s Service (LPC, 2019)：https://linuxplumbersconf.org/event/4/contributions/458/
- 基于 BPF/XDP 实现 K8s Service 负载均衡 (LPC, 2020)https://linuxplumbersconf.org/event/7/contributions/674/
- 深入理解 Kubernetes 网络模型：自己实现 Kube-Proxy 的功能: https://cloudnative.to/blog/k8s-node-proxy/
- Containerd 使用教程：https://icloudnative.io/posts/getting-started-with-containerd/
- Kubernetes 的层级命名空间介绍：https://icloudnative.io/posts/introducing-hierarchical-namespaces/
- Kubernetes 的设计理念：https://jimmysong.io/kubernetes-handbook/concepts/concepts.html
- Kubernetes 部署教程：https://zhuanlan.zhihu.com/p/641521752





