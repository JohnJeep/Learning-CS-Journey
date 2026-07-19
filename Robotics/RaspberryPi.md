<!--
 * @Author: JohnJeep
 * @Date: 2026-07-19 11:53:02
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-07-19 23:51:08
 * @Description: Raspberry Pi related content for Robotics.
 * Copyright (c) 2026 by John Jeep, All Rights Reserved. 
-->


## 0.1. Raspberry Pi Zero W

产品规格

- **1GHz, single-core CPU Broadcom BCM2835, 512MB RAM**
- 802.11 b/g/n wireless LAN
- Bluetooth 4.1
- Bluetooth Low Energy (BLE)
- Mini HDMI® port and micro USB On-The-Go (OTG) port
- Micro USB power
- HAT-compatible 40-pin header
- Composite video and reset headers
- CSI camera connector



注意点

- ARMv6 架构；

- Pi Zero W 用的是 Broadcom BCM43438 无线芯片，**是个非开源二进制固件 + nvram 校准文件的组合**，缺一不可；





场景：

- Zero W 用于某个子系统的小控制节点；





## 0.2. Raspberry Pi 3 Model B+

The Raspberry Pi 3 Model B+ is the final revision in the Raspberry Pi 3 range.

- **Broadcom BCM2837B0** (4 core), Cortex-A53 (ARMv8) 64-bit SoC @ 1.4GHz
- 1GB LPDDR2 SDRAM
- 2.4GHz and 5GHz IEEE 802.11.b/g/n/ac wireless LAN, Bluetooth 4.2, BLE
- Gigabit Ethernet over USB 2.0 (maximum throughput 300 Mbps)
- Extended 40-pin GPIO header
- Full-size HDMI®
- 4 USB 2.0 ports
- CSI camera port for connecting a Raspberry Pi camera
- DSI display port for connecting a Raspberry Pi touchscreen display
- 4-pole stereo output and composite video port
- Micro SD port for loading your operating system and storing data
- 5V/2.5A DC power input
- Power-over-Ethernet (PoE) support (requires separate PoE HAT)



ARMv7 架构；



## 0.3. 交叉构建Buildroot

|                     | Pi 3B+                                       | Pi Zero W                 |
| ------------------- | -------------------------------------------- | ------------------------- |
| CPU                 | 四核 Cortex-A53 (armv7代码兼容，实际是armv8) | 单核 ARM11 (armv6)        |
| Buildroot defconfig | `raspberrypi3_defconfig`                     | `raspberrypi0w_defconfig` |
| 架构差异            | armv7 (32位常用配置)                         | **armv6**，工具链不通用   |

**关键点**：Pi Zero W 用的是 ARM1176（armv6），跟 Pi 3B+ 的 Cortex-A53（armv7/v8）指令集不完全兼容。你不能编译一份镜像烧两块板子，**必须为每块板子单独跑一次 Buildroot 配置和编译**，产出两个独立的 `sdcard.img`。

1、构建 Dockerfile

```dockerfile
FROM ubuntu:24.04

RUN apt-get update && apt-get install -y \
    git build-essential libncurses-dev bc bison flex \
    libssl-dev cpio rsync unzip wget file python3 sudo \
    && rm -rf /var/lib/apt/lists/*

RUN useradd -ms /bin/bash builder
USER builder
WORKDIR /home/builder

RUN git clone https://github.com/buildroot/buildroot.git
WORKDIR /home/builder/buildroot
```

2、构建镜像并进入容器

```bash
docker build -t rpi3-buildroot .

docker run -it --rm \
  -v $(pwd)/output:/home/builder/buildroot/output \
  rpi3-buildroot bash
```

`-v` 把容器内的 `output` 目录挂载到宿主机当前目录下的 `output`，编译产物直接落地在宿主机上，容器退出也不丢。

同时维护两块板子的构建，配置容易搞混，用一个目录结构管理：

```
rpi-build/
├── Dockerfile
├── dl/                    # 共用下载缓存
├── configs/
│   ├── 3b.config          # 3B+ 的 defconfig 备份
│   └── zerow.config       # Zero W 的 defconfig 备份
├── output-3b/
└── output-zerow/
```

3、分别启动不同的容器，给2块板子执行构建流程

Pi 3B+：

```bash
docker run -it --rm \
  -v D:/rpi-build/output-3b:/home/builder/buildroot/output \
  -v D:/rpi-build/dl:/home/builder/buildroot/dl \
  rpi-buildroot bash

# 容器内
make raspberrypi3_defconfig
make menuconfig          # 可选调整
make busybox-menuconfig  # 裁剪applet
make -j$(nproc)
```

Pi Zero W：

```bash
docker run -it --rm \
  -v D:/rpi-build/output-zerow:/home/builder/buildroot/output \
  -v D:/rpi-build/dl:/home/builder/buildroot/dl \
  rpi-buildroot bash

# 容器内
make raspberrypi0w_defconfig
make -j$(nproc)
```

编译完成后退出容器，宿主机的 `./output/images/sdcard.img` 就是最终产物。



**关键注意点**

1. **`useradd builder` 这一步是必须的**——Buildroot 官方明确不允许以 root 用户执行编译，容器默认是 root，会直接报错退出。这是最容易踩的坑。
2. **`-j$(nproc)` 用的是容器可见的核数**，如果 Docker Desktop（Mac/Windows）限制了容器CPU数，编译会变慢，可以在 Docker 设置里调高。
3. **持久化 dl 目录**（Buildroot 下载源码包的缓存目录），避免每次重新 build 容器都要重新下载全部源码;
4. **网络问题**：Buildroot 编译过程要从 kernel.org、各个开源项目的服务器下载源码，如果你的 Docker 环境网络受限（比如公司代理），需要在 Dockerfile 或 `docker run` 里配置好 `http_proxy`/`https_proxy`。
5. **写卡这一步必须在宿主机上做**，不要试图在容器里操作 `/dev/sdX`——除非用 `--privileged` 且映射设备进去，一般没这个必要，直接容器外 `dd` 更简单安全。

4、将构建好的产物烧录到板子上；



# 1. References

- https://www.raspberrypi.com/documentation/computers/raspberry-pi.html

```
docker run -it --rm -v /i/ubuntu2404/workspace/rpi-build/output-3b:/home/builder/buildroot/output  -v /i/ubuntu2404/workspace/rpi-build/dl:/home/builder/buildroot/dl rpi-buildroot bash
```

