## Gazebo

Gazebo 是一个仿真工具，提供逼真的仿真环境。



## 版本

| Platform           | Gazebo Versions                                              |
| ------------------ | ------------------------------------------------------------ |
| Ubuntu 24.04 Noble | [Gazebo Jetty](https://gazebosim.org/docs/jetty/install_ubuntu) (recommended), [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install_ubuntu) (recommended if using ROS 2 Jazzy) and [Gazebo Ionic](https://gazebosim.org/docs/ionic/install_ubuntu) |
| Ubuntu 22.04 Jammy | [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install_ubuntu) (recommended) and [Gazebo Fortress](https://gazebosim.org/docs/fortress/install_ubuntu) (recommended if using ROS 2 Humble) |
| Mac Ventura        | [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install_osx) (recommended) and [Gazebo Fortress](https://gazebosim.org/docs/fortress/install_osx) |
| Mac Monterey       | [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install_osx) (recommended) and [Gazebo Fortress](https://gazebosim.org/docs/fortress/install_osx) |
| Windows            | Support via Conda-Forge is not fully functional, as there are known runtime issues [see this issue for details](https://github.com/gazebosim/gz-sim/issues/168). |

[SDFormat](http://sdformat.org/) (Simulation Description Format), sometimes abbreviated as SDF, is an XML format that describes objects and environments for robot simulators, visualization, and control.

**请注意，从Ubuntu 24.04和ROS Jazzy开始，官方默认的不再是旧的"Gazebo Classic"（如gazebo11），而是新的Gazebo Gz系列（此处为Harmonic）。它们在使用命令上有所不同，例如新版本使用 `gz sim` 来启动。**



## Install

Binary Installation on Ubuntu

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz*    // 所有包都安装上

sudo apt install curl lsb-release gnupg

sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update
sudo apt install gz-harmonic
```

查看版本

```bash
gz sim --version
Gazebo Sim, version 8.9.0
Copyright (C) 2018 Open Source Robotics Foundation.
Released under the Apache 2.0 License.

```

运行

```bash
gz sim
```

卸载

```bash
sudo apt remove gz-harmonic && sudo apt autoremove
```



## Plugins

两轮差速器：ros-jazzy-gazebo-plugins



## 用法

模型（model）、链接（link）、关节（joint）、传感器（sensor）



模型

模型可以是一个机器人、一张桌子、一个盒子等。每个模型由一个或多个链接（link）组成，链接之间通过关节（joint）连接。每个链接可以具有多个碰撞（collision）和视觉（visual）属性，以及惯性（inertial）属性。

- 单个模型（model）通常保存为.sdf文件，其中包含一个`<model>`标签（但也可以包含`<world>`，不过通常单个模型文件只包含`<model>`）。

- 场景（world）文件通常保存为`.world`，其中包含一个`<world>`标签，里面可以有多个模型、光照等。

- 每个 `.world` 文件只能有一个 `<world>` 标签：



## 为什么只能有一个 world？

- **设计原则**：每个 SDF 文件代表一个完整的仿真环境
- **逻辑分离**：不同的仿真场景应该保存在不同的 `.world` 文件中
- **Gazebo 限制**：Gazebo 一次只能加载一个世界

一个 world 可以包含多个模型（model），但是一个SDF文件通常只包含一个世界。在SDF的顶层结构中，我们使用一个`<world>`标签来定义一个世界，然后在这个世界中可以包含多个模型。





































## References

Gazebo harmonic: https://gazebosim.org/docs/harmonic/install_ubuntu/