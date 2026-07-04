<!--
 * @Author: JohnJeep
 * @Date: 2026-07-04 11:26:57
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-07-04 17:31:34
 * @Description: rosdep usage
 * Copyright (c) 2026 by John Jeep, All Rights Reserved. 
-->

# 1. rosdep 是做什么的？

`rosdep` 是 ROS/ROS2 生态里的**系统依赖管理工具**。它本身不管理 ROS 包之间的依赖（那是 `colcon`/`ament` 和 `package.xml` 里的 `<depend>` 标签负责的事），而是专门解决一个问题：

**你的 ROS 包依赖的那些"系统级"第三方库（比如 Boost、OpenCV、Eigen、某个 apt 包），在当前这台机器上要怎么装？**

## 1.1. 核心思路

每个 ROS 包的 `package.xml` 里会声明依赖，比如：

```xml
<depend>eigen</depend>
<depend>libopencv-dev</depend>
<exec_depend>python3-numpy</exec_depend>
```

这里的 `eigen`、`libopencv-dev` 是**rosdep key**（一种抽象名字），不是具体某个发行版的包名。因为同一个依赖在 Ubuntu 上叫 `libeigen3-dev`，在 Fedora 上可能叫别的名字，在 macOS 上用 brew 装又是另一个名字。

`rosdep` 维护了一份**映射表**（rosdep 数据库，来自 `rosdistro` 仓库），把这些抽象 key 翻译成具体操作系统上的具体包管理器命令。



# 2. rosdep 使用指南

## 2.1. 安装

```bash
# Ubuntu/Debian 系统，通常 ROS2 装好后就有
sudo apt install python3-rosdep
```

## 2.2. 初始化（每台机器只需一次）

```bash
sudo rosdep init
```

这会在 `/etc/ros/rosdep/sources.list.d/20-default.list` 写入默认的数据源配置（指向官方 `rosdistro` 仓库）。如果已经初始化过会提示已存在，忽略即可。

## 2.3. 更新数据库（建议定期做）

```bash
rosdep update
```

这会从网络拉取最新的 rosdep key 映射表，缓存到 `~/.ros/rosdep/`。**每次系统依赖表有更新，或者换了新机器，都要重新跑这个。**

## 2.4. 核心用法：安装依赖

在你的 workspace 根目录（包含 `src/` 的那一层）执行：

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

这是最常用的一条命令，效果是：扫描 `src` 下所有包的 `package.xml`，把里面声明的依赖翻译成 apt 包名并自动安装。

### 2.4.1. 常用参数

| 参数                 | 作用                                                  |
| -------------------- | ----------------------------------------------------- |
| `--from-paths src`   | 指定扫描哪些路径（可以给多个路径）                    |
| `--ignore-src`       | 跳过那些本身也在 `src` 里源码编译的兄弟包，避免重复装 |
| `-r`                 | 某个包解析失败时继续处理其他的，不中断整体流程        |
| `-y`                 | 自动确认，无需交互（脚本/CI 里必备）                  |
| `--simulate`         | 只打印会执行的命令，不真的安装（先看看会装啥）        |
| `--rosdistro humble` | 显式指定发行版（一般会自动读环境变量 `ROS_DISTRO`）   |

先跑一遍模拟看看会装什么，比较稳妥：

```bash
rosdep install --from-paths src --ignore-src -r -y --simulate
```

## 2.5. 调试单个依赖

如果某个包依赖装不上，想单独查一下这个 key 到底映射成了什么：

```bash
# 查看某个 rosdep key 在当前系统上对应哪个具体包
rosdep resolve <key_name>

# 例如
rosdep resolve eigen
# 输出类似：
# #apt
# libeigen3-dev
```

如果提示找不到 key，说明这个依赖没有在 rosdep 数据库里注册，可能需要自定义映射（见下面第 6 点）。

## 2.6. 自定义依赖映射（遇到私有/内部包时用）

如果你的 `package.xml` 里声明的依赖是公司内部的，官方 rosdep 数据库里当然查不到，会报错类似 `Cannot locate rosdep definition for [xxx]`。

解决办法是自己写一个 yaml 映射文件，比如 `my_rosdep.yaml`：

```yaml
cv_task_msgs:
  ubuntu:
    apt:
      packages: [ros-humble-cv-task-msgs]
```

然后注册到 sources list：

```bash
echo "yaml file:///path/to/my_rosdep.yaml" | sudo tee /etc/ros/rosdep/sources.list.d/10-my-packages.list
rosdep update
```

之后 `rosdep resolve cv_task_msgs` 就能正常解析了。

## 2.7. 典型工作流（clone 一个新项目后）

```bash
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

## 2.8. Docker/CI 里的常见写法

```dockerfile
COPY src/ /ros2_ws/src/
RUN rosdep update && \
    rosdep install --from-paths /ros2_ws/src --ignore-src -r -y
```

**注意顺序**：一定要先 `COPY` 源码（保证 `package.xml` 都在），再跑 `rosdep install`，最后再 `colcon build`。


# 3. References

- https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Rosdep.html

