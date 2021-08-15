<!--
 * @Author: JohnJeep
 * @Date: 2021-04-07 23:25:09
 * @LastEditTime: 2021-08-15 16:42:08
 * @LastEditors: Windows10
 * @Description: In User Settings Edit
-->

# 1. 概念
- 记录编译记录的文件。
- 名称两种规则：全小写（makefile）或首字母大写（Makefile）
- 直接使用 `make` 指令，会生成Makefile文件中定义的最终目标文件。
- 使用 `make 自定义变量名`，会执行自定义变量名下面定义的规则指令。


# 2. 三个基本要素
- 目标
- 依赖
- 命令

<div align="center"> 
  <img width="80%" height="80%" src="../pictures/makefile三要素.png" />
</div>
<div align="center">
  <img width="80%" height="80%" src="../pictures/makefile工作原理-1.png" />
</div>
<div align="center"> 
  <img width="80%" height="80%" src="../pictures/makefile工作原理-2.png" />
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

# 3. 参考
- [CMake 入门实战](https://www.hahack.com/codes/cmake/)
- [CMake教程](https://blog.csdn.net/fan_hai_ping/article/details/42524205)
- [在 linux 下使用 CMake 构建应用程序](https://www.ibm.com/developerworks/cn/linux/l-cn-cmake/index.html)
- [CMake官网](http://www.cmake.org/)
- [cmake常用命令](https://cmake.org/cmake/help/v2.8.8/cmake.html#section_Commands)
- [Cmake大型项目设置指南](https://oldpan.me/archives/cmake-meta-project-use)
- [CLoin与CMake详细教程](https://www.jetbrains.com/help/clion/quick-tutorial-on-configuring-clion-on-windows.html)
- [VSCode运行多文件C++教程：使用CMake](https://blog.csdn.net/frostime/article/details/86756983)