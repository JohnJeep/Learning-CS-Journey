<!--
 * @Author: JohnJeep
 * @Date: 2023-05-27 11:08:34
 * @LastEditors: JohnJeep
 * @LastEditTime: 2023-05-27 15:33:00
 * @Description: CPU 学习记录
 * Copyright (c) 2023 by John Jeep, All Rights Reserved. 
-->

Linux 终端下运行 `lscpu` 命令查看关于 CPU 架构的信息。

```shell
[root@AP03 log]# lscpu
Architecture:          x86_64
CPU op-mode(s):        32-bit, 64-bit
Byte Order:            Little Endian
CPU(s):                8
On-line CPU(s) list:   0-7
Thread(s) per core:    1
Core(s) per socket:    2
Socket(s):             4
NUMA node(s):          1
Vendor ID:             GenuineIntel
CPU family:            6
Model:                 15
Model name:            Intel(R) Xeon(R) CPU E5-2620 v4 @ 2.10GHz
Stepping:              1
CPU MHz:               2099.998
BogoMIPS:              4199.99
Hypervisor vendor:     VMware
Virtualization type:   full
L1d cache:             32K
L1i cache:             32K
L2 cache:              256K
L3 cache:              20480K
NUMA node0 CPU(s):     0-7
Flags:                 fpu vme de pse tsc msr pae mce cx8 apic sep mtrr pge mca cmov pat pse36 clflush dts mmx fxsr sse sse2 ss ht syscall nx lm constant_tsc arch_perfmon pebs bts nopl tsc_reliable nonstop_tsc eagerfpu pni ssse3 cx16 x2apic tsc_deadline_timer hypervisor lahf_lm tsc_adjust arat
```

对上面的显示结果进行解释：

- Architecture: 表示系统的架构，这里是 x86_64，表示 64 位的 x86 架构。
- CPU op-mode(s): 表示CPU支持的操作模式，这里支持32位和64位操作模式。
- Byte Order: 表示系统的字节序，这里是Little Endian，表示低位字节存储在内存的低地址。
- CPU(s): 表示CPU的总核心数，这里是8个核心。
- On-line CPU(s) list: 表示在线的CPU核心列表，从0到7。
- Thread(s) per core: 表示每个核心的线程数，这里是1，表示没有使用超线程技术。
- Core(s) per socket: 表示每个CPU插槽的核心数，这里是2个核心。
- Socket(s): 表示CPU插槽的数量，这里是4个插槽。
- NUMA node(s): 表示NUMA（非一致性内存访问）节点的数量，这里是1个节点。
- Vendor ID: 表示CPU制造商的标识，这里是GenuineIntel，表示英特尔的CPU。
- CPU family: 表示CPU家族，这里是6。
- Model: 表示CPU的型号，这里是15。
- Model name: 表示CPU的型号名称，这里是Intel(R) Xeon(R) CPU E5-2620 v4 @ 2.10GHz。
- Stepping: 表示CPU的步进号，这里是1。
- CPU MHz: 表示CPU的时钟频率，这里是2099.998 MHz。
- BogoMIPS: 表示CPU的BogoMIPS值，这是一个用于性能比较的指标。
- Hypervisor vendor: 表示虚拟化平台的供应商，这里是VMware。
- Virtualization type: 表示虚拟化的类型，这里是full，表示完全虚拟化。
- L1d cache: 表示CPU的一级数据缓存大小，这里是32KB。
- L1i cache: 表示CPU的一级指令缓存大小，这里是32KB。
- L2 cache: 表示CPU的二级缓存大小，这里是256KB。
- L3 cache: 表示CPU的三级缓存大小，这里是20480KB。
- NUMA node0 CPU(s): 表示NUMA节点0上的CPU核心列表，从0到7。
- Flags: 表示CPU的特性标志，包括对特定指令集的支持等。

说明：

1. 对 Socket 的理解：术语 "Socket" 在计算机网络中通常用于描述套接字（socket），表示网络通信的端点。套接字是一种应用程序接口（API），用于在网络中建立通信连接。在CPU上指的是CPU插槽接口，英文名称为：CPU socket 或 CPU slot。是一种安装CPU的物理接口，提供了电力和信号连接，这些插槽通常具有特定的形状和布局，以适应特定类型的CPU。