---
title: microsoft-wsl
data: 2025-03-30 00:04:10
tags: ['Linux']
category: Linux
---

<!--
 * @Author: JohnJeep
 * @Date: 2022-04-07 14:19:12
 * @LastEditTime: 2022-04-07 14:50:42
 * @LastEditors: DESKTOP-0S33AUT
 * @Description: Microsoft wsl 用法
-->

# 1. WSL

## 1.1. 安装

Microsoft 官方 docs: https://docs.microsoft.com/zh-cn/windows/wsl/



## 1.2. WSL 中修改 hostname

默认情况下 wsl 的 hostname 是和当前 windows 系统的主机名称保持一致的，有时候 Windows 系统的主机名太长了，
在 wsl 中显示太长，感觉不是很舒服，觉得有洁癖，但又不好改 windows 的主机名，那么只能改 wsl 的主机名了。
在 wsl 下使用 `/etc/hostname`  命令修改主机名时，发现并不能完全修改，
在重新进入 wsl 后又会恢复成原来的样子，发现这种方式在 wsl 中不能永久生效。因此需要通过另外一种方式来改变。

进入 `/etc` 目录， 编辑 `wsl.conf`，如果没有该文件就创建一个，在文件中加入下面的内容

```bash
[user]
default=root

[network]
generateHosts=false
hostname=WSL2
```

- `user` 组下面，`default` 后面的数值表示进入 wsl 后默认启动的用户，当前设置为 root；
- `network` 组下面，`generateHosts` 表示是否自动生成 `hosts` 文件，`hostname` 表示 wsl 主机名。

设置完参数后保存退出， window 终端执行 `wsl --shutwown` 关闭 wsl，然后用 `wsl` 命令启动系统，进入 wsl 系统后，发现 `hostname` 已被修改为 `WSL2`。



## 1.3. wsl 下安装 zsh

- 在WSL中安装zsh终端：https://www.cnblogs.com/hongdada/p/11087557.html
- oh my zsh github themes: https://github.com/ohmyzsh/ohmyzsh/wiki/Themes
- agnoster 主题安装 powerline 字体: https://github.com/powerline/fonts


## 1.4. Trouble Shooting

### 1.4.1. WSL莫名奇怪不能连接网络

终端执行下面的命令，重启机器，解决网络问题。
```bash
wsl --shutdown
netsh winsock reset
netsh int ip reset all
netsh winhttp reset proxy
ipconfig /flushdns
```



# References

- [Development Containers](https://containers.dev/)
- https://www.youtube.com/@code/videos