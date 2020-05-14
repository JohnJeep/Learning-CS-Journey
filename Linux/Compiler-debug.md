```
 * @Author: your name
 * @Date: 2020-04-23 20:37:04
 * @LastEditTime: 2020-05-14 23:12:19
 * @LastEditors: your name
 * @Description: In User Settings Edit
```
### 调试GDB
- 使用 gdb 调试程序之前,必须使用 `-g` 或 `–ggdb`编译选项编译源文件。







### 编译
1. CMake




2.  汇编语法



--------------------------------------------------

1.可调试gcc编译：gcc -g -o xxx xxx.c

2.启动gdb调试

gdb xxx

3.在main函数处设置断点

break main

4.运行程序

run

5.其他调试命令

list（l）查看程序

break(b) 函数名：在某函数入口处添加断点

break  行号：在指定行添加断点

break  文件名：行号   在指定文件的指定行添加断点

info  break  查看所有设置的断点

delete 断点编号  删除断点编号的断点

--------------------------------------------------

next(n)  :单步运行程序（但不进入子函数）

step(s)  :单步运行程序（进入子函数）

continue（c）：继续运行程序

print（p）变量名: 查看指定变量值

set var=value ：设置变量的值

quit(q):退出gdb

