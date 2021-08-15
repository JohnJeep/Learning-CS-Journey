<!--
 * @Author: JohnJeep
 * @Date: 2021-04-17 18:15:50
 * @LastEditTime: 2021-04-18 21:53:28
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
-->
## 4 Layer OSI Model

![4-layer-model](../pictures/4-layer-model.png)

## 7 Layer OSI Model

![7-layer-OSI-model](../pictures/7-layer-OSI-model.png)

![ip-is-the-thin-waist](../pictures/ip-is-the-thin-waist.png)

## IP

![Internet-Protocol](../pictures/Internet-Protocol.png)

![IP-service-Model-1](../pictures/IP-service-Model-1.png)

![IP-service-Model-2](../pictures\IP-service-Model-2.png)

Why is the IP service so simple?

1.  让网络保持简单、最小化。更快、更简化的特性，使其降低维护成本，不需要经常升级。
2.  端到端原则（end-to-end principle）：尽可能正在端主机中实现功能。
3.  允许在顶部构建各种可靠（或不可靠）的服务。
4.  工作于任何的链路层（link layer）：IP对下层链路层的期望很小，连接可以是无线的，也可以是有线的，不需要重传和拥塞控制。

**什么是Reliable byte stream?（可靠的字节流）**

>  Sequence of bytes (in each direction) delivered in order, correctly

RPC: Remote Procedure Call

更深入的细节

1.  IP尝试阻止数据包永远循环。因为IP 路由器通过Internet逐跳转发数据包。