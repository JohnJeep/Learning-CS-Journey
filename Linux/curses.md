<!--
 * @Author: JohnJeep
 * @Date: 2022-01-27 17:21:53
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-11-20 12:00:11
 * @Description: linux curses usage
 * Copyright (c) 2024 by John Jeep, All Rights Reserved. 
-->

- [1. 介绍](#1-介绍)
- [2. 安装](#2-安装)
  - [2.1. Ubuntu 中安装 curses](#21-ubuntu-中安装-curses)
  - [2.2. centos 中安装 curses](#22-centos-中安装-curses)
  - [2.3. 源码安装 ncurse 库](#23-源码安装-ncurse-库)
- [3. 案例](#3-案例)
- [4. References](#4-references)


# 1. 介绍

[ncurses](https://link.zhihu.com/?target=http%3A//www.gnu.org/software/ncurses/ncurses.html)(new curses)是一套编程库，它提供了一系列的函数以便使用者调用它们去生成基于文本的用户界面。
ncurses名字中的**n**意味着“new”，因为它是**curses**的自由软件版本。由于AT&T“臭名昭著”的版权政策，人们不得不在后来用ncurses去代替它。
ncurses是[GNU计划](https://link.zhihu.com/?target=https%3A//en.wikipedia.org/wiki/GNU_Project%20GNU%E8%AE%A1%E5%88%92)的一部分，但它却是少数几个不使用GNU GPL或LGPL授权的GNU软件之一。

其实我们对ncurses本身并不陌生，以下几款大名鼎鼎的软件都用到过ncurses：

- vim
- emacs
- lynx
- screen

# 2. 安装

## 2.1. Ubuntu 中安装 curses

```
sudo apt-get install libncurses5-dev
```

## 2.2. centos 中安装 curses

```
yum install ncurses-devel
```

## 2.3. 源码安装 ncurse 库

下载地址：http://ftp.gnu.org/pub/gnu/ncurses/



# 3. 案例

示例：生成一个基于文本的图像化界面。

```c
#include <string.h>
#include <ncurses.h>

int main(int argc,char* argv[]){
    initscr();
    raw();
    noecho();
    curs_set(0);

    char* ptr = "Hello, StephenWen!";

    mvprintw(LINES/2,(COLS-strlen(ptr))/2,ptr);
    refresh();

    getch();
    endwin();

    return 0;
}
```



# 4. References

- https://www.cnblogs.com/memoryXudy/p/10830548.html
