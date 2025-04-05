<!--
 * @Author: JohnJeep
 * @Date: 2025-04-01 00:40:42
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-04-05 11:21:42
 * @Description: 
 * Copyright (c) 2025 by John Jeep, All Rights Reserved. 
-->
# Introduction

jq 是 stedolan 开发的一个轻量级的和灵活的命令行 JSON 处理器。

它主要用于在命令行界面处理 JSON 输入，并使用给定的过滤条件来过滤符合条件的新的 JSON 串。

通常在类 Unix 环境下，我们可以快速的使用 `jq` 来进行 JSON 数据格式化过滤和处理。同时需要注意的是，该命令行工具和 awk/sed/grep 工具一样，属于系统的默认命令，如果系统没有该命令，可以尝试使用如下方式进行安装。

```shell
# Ubuntu 系列
$ sudo apt-get install jq 

# CentOS 系列
$ yum install jq 
```



# References

- 官方文档：https://stedolan.github.io/jq/
- 知乎介绍 jq 用法：https://zhuanlan.zhihu.com/p/606945462
- 个人（上月行）博客介绍 jq 用法：https://shanyue.tech/op/jq.html