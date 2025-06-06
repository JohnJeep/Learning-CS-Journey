<!--

 * @Author: JohnJeep
 * @Date: 2021-04-07 23:25:09
 * @LastEditTime: 2022-05-12 09:02:36
 * @LastEditors: DESKTOP-0S33AUT
 * @Description: In User Settings Edit
-->

<!-- TOC -->

- [1. Makefile](#1-makefile)
  - [1.1. 什么是 Makefile?](#11-什么是-makefile)
  - [1.2. 什么是 Make 和 configure ？](#12-什么是-make-和-configure-)
  - [1.3. 检测程序会检测哪些内容？](#13-检测程序会检测哪些内容)
  - [1.4. 源码下载](#14-源码下载)
  - [1.5. 操作步骤](#15-操作步骤)
  - [1.6. 三个基本要素](#16-三个基本要素)

<!-- /TOC -->


# 1. Makefile

## 1.1. 什么是 Makefile?

它是记录编译记录的文件。有两种命名的方式：全小写 makefile 或首字母大写 Makefile。


## 1.2. 什么是 Make 和 configure ？

make 是一个程序，会去找 Makefile ，那 Makefile 怎么写？ 通常软件开发商都会写一个检测程序来侦测使用户的操作环境， 以及操作环境是否有软件开发商所需要的其他功能，该检测程序检测完毕后， 就会主动的建立这个 Makefile 的规则文件， 通常这个检测程序的文件名为 configure 或者是 config 。



## 1.3. 检测程序会检测哪些内容？

- 是否有合适的编译可以编译本软件的代码
- 是否存在有本软件所需要的函数库，或其它的所需的依赖软件
- 操作系统是否适合本软件，包括 Linux 的内核
- 内核的头文件（header include）是否存在，驱动程序必须要的检测


## 1.4. 源码下载

同一个软件在不同的平台上执行，需要重新编译，这就是为什么在下载类 Linux 的软件程序时，会提供源码下载。



## 1.5. 操作步骤

- 直接使用 `make` 指令，会生成 Makefile 文件中定义的最终目标文件。
- 使用 `make 自定义变量名`，会执行自定义变量名下面定义的规则指令。


## 1.6. 三个基本要素
- 目标
- 依赖
- 命令

<div align="center"> 
  <img width="80%" height="80%" src="../figures/makefile三要素.png" />
</div>
<div align="center">
  <img width="80%" height="80%" src="../figures/makefile工作原理-1.png" />
</div>
<div align="center"> 
  <img width="80%" height="80%" src="../figures/makefile工作原理-2.png" />
</div>


- 一个规则
- 两个函数。每个函数都有返回值
  - `src= $(wildcard ./*c)` 查找指定目录 `./` 下所有 `.c` 的文件，并将函数的返回值赋值给 src 变量
  - 匹配替换函数  `obj = $(patsubst ./%.c, ./%.o, $(src))` 将指定目录 `./` 下所有的 `.c` 替换为 `.o`文件

- 三个变量
  - 自定义变量
  - 自动变量
    - `$<` 规则中的第一个依赖
    - `$@` 规则中的目标
    - `$^` 规则中的所有依赖
  - 系统维护的变量(一般为大写字符) 
    - `CPPFLAGS` 预处理所需要的的选项。如：`-I`
    - `CFLAGS  ` 编译时使用的参数。`-Wall, -g, -c`
    - `LDFLAGS ` 链接库使用的选项。`-L -l(小写)`
    - `CC` 等于gcc

- 伪目标 `.PHONY` 
  - `.PHONY: clean`
  - `-` 表示当前指令执行不成功则忽略当前指令。

- 模式规则
  ```makefile
  // 给相同的命令指定一个规则
  %.o: %c
      gcc -c $< -o $@
  ```
