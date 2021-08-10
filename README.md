<!--
 * @Author: JohnJeep
 * @Date: 2020-04-04 09:46:51
 * @LastEditTime: 2021-08-10 16:52:57
 * @LastEditors: Windows10
 * @Description: In User Settings Edit
-->
# 1. Learning Computer Science Journey

本项目是记录本人学习计算机科学这类学科知识点的历程。从计算机底层硬件到应用层软件等知识点。包括最底层的汇编语言（Assemble）、高级的C、C++、Go语言、计算机网络基础（ComputerNetwork）、操数据结构与算法（DataStructure）、设计模式（DesignPattern）、操作系统（OS）、Linux基础与Linux环境编程（Linux）、数据库（MySQL、Redis）、脚本语言（Shell）、界面编程（Qt）、轻量级的Web服务器（Nginx）、GitHub和Git及SVN代码管理工具使用（Git-SVN）、剑指offer及LeetCode刷题经验（Interview）、嵌入式系统中STM系列芯片模板工程的创建（Embedded）、常用的学习工具和学习网站之谈（StudyTool）、Markdown和Jupyter Notebook基本语法（markdown）、基本的一些哲学思想和英语美句（Philosophy）。



# 2. C 语言
C 语言是自己接触的第一门语言，基本的语法在大学时就已经学过，这里不再讲基础的语法，仅仅只是记录自己在工作中对 C 语言知识点的补充。包括底层的一些知识点，字符编码、指针、函数指针、结构体、内存分配等常见的点。

[字符编码](C/2-字符编码.md)

[内存存放顺序](C/3-内存存放顺序.md)

[断点调试](C/4-断点调试.md)

[变量与数据类型](C/5-变量与数据类型.md)

[Volatile 关键字用法解释](C/6-volatile.md)

[为什么会出现段错误](C/7-segment-fault.md)

[C 语言中常见的基础语法的补充](C/8-基础知识补充.md)

[C 语言中精髓之一：指针函数和函数指针](C/9-指针函数和函数指针.md)

[Const 关键字](C/10-const.md)

[restrict 关键字](C/11-restrict.md)

[exteren 关键字](C/12-extern.md)

[最常见得 void 含义和用法](C/13-void.md)

[内存初始化函数 memset](C/14-memset.md)

[memcpy 与 strcpy 函数的区别及用法](C/15-strncpy与memcpy.md)

[memcmp 与 strcmp 函数的区别及用法](C/16-memcmp与strcmp.md)

[typedef 用法剖析](C/17-typedef.md)

[C 语言中精髓之一：结构体和字节对齐](C/18-struct和字节对齐.md)

[word, half word, double word用法及区别](C/20-字、半字、双字.md)

[C 语言中精髓之一：指针](C/21-指针.md)

[枚举类型](C/22-枚举类型(Enumerations).md)

[C 语言中的状态机](C/23-状态机.md)

[C 语言中精髓之一：数组与指针的结合](C/24-数组与指针组合.md)

用到的比较生僻的库函数：[atoi()](C/25-atoi().md)、 [fprintf()](C/26-fprintf().md)

[常见的转义字符和ASCII码](C/29-转义字符与ASCII码.md)

[C 语言中最重要的部分：内存分布](C/30-内存.md)



# 3. C++
这部分主要是C++

[C++基础](./C++/01-C++基础.md)

[C++高级](C++/02-C++高级.md)

[C++新特性](C++/03-C++新特性.md)

[C++STL](C++/04-STL.md)

[C++ 内存管理](./C++/05-内存管理.md)

[Core dump调试使用](./C++/Core-dump.md)

[Valgrind内存泄漏检查的利器](C++/Valgrind.md)

[编程规范](./C++/编程规范.md)


# 4. 内功修炼
- B站：[卡内基·梅隆大学-CSAPP](https://www.bilibili.com/video/BV1iW411d7hd)





# 5. 操作系统
## 5.1. MIT 6.828 & 6.S081
- 课程
  - MIT: CS8.828, [Operating System Engineering](https://pdos.csail.mit.edu/6.828/2018/schedule.html) 2018年秋季学科大纲。

- 视频
  - YouTube：[6.828](https://www.youtube.com/playlist?list=PLfciLKR3SgqNJKKIKUliWoNBBH1VHL3AP)
  - B站：[6.828](https://www.bilibili.com/video/av15896196/)
  - B站：[6.S081](https://www.bilibili.com/video/BV19k4y1C7kA?from=search&seid=8656595108283984685) 
> NOte: 6.828 and 6.S081 will be offered as two separate classes. 6.S081 (Introduction to Operating Systems) will be taught as a stand-alone AUS subject for undergraduates, and will provide an introduction to operating systems. 6.828 will be offered as a graduate-level seminar-style class focused on research in operating systems. 6.828 will assume you have taken 6.S081 or an equivalent class. See the [6.828 web site](https://abelay.github.io/6828seminar/schedule.html) for more detail about 6.828.

- 他人课程笔记
  - [知乎：MIT6.828-神级OS课程-要是早遇到，我还会是这种 five 系列](https://zhuanlan.zhihu.com/p/74028717) 
  - [二十八画生征友：一起来通关6.S081/6.828吧](https://zhuanlan.zhihu.com/p/251366985)

## 5.2. Operating Systems: Three Easy Pieces
- [操作系统导论(Operating Systems: Three Easy Pieces)](http://pages.cs.wisc.edu/~remzi/OSTEP/) 一本非常好的书籍，并且电子版本是免费的。



## 5.3. 清华大学OS
- [清华大学操作系统课程(2019)](https://chyyuu.gitbooks.io/os_course_info/content/)：Gitbook上清华大学操作系统课程(2019)。
- [操作系统(Operating Systems) (2019)](http://os.cs.tsinghua.edu.cn/oscourse/OS2019spring): 清华大学2019学期操作系统课程主页。
- [操作系统(Operating Systems) (2020)](http://os.cs.tsinghua.edu.cn/oscourse/OS2020spring): 清华大学2020学期操作系统课程主页。
- [uCore OS 实验指导书和源码网址 (2020)](https://learningos.github.io/ucore_os_webdocs/):清华大学2020学期uCore OS 实验指导书。
- [Operating System Concepts Ninth Edition](https://www.os-book.com/OS9/): 操作系统概念第九版英文版在线主页。


# 6. 计算机网络
## 6.1. 斯坦福大学CS144
- 课程官方网站  
  - [CS 144: Introduction to Computer Networking, Fall 2020](https://cs144.github.io/)：为主。
  - [CS144: Introduction to Computer Networking, Fall 2010](https://www.scs.stanford.edu/10au-cs144/): 可选。


- 书籍
  - 《计算机网络：自顶向下方法》
  - [Computer Networking: a Top Down Approach 8th edition](http://gaia.cs.umass.edu/kurose_ross/):《计算机网络：自顶向下方法》原书的英文版，最新版本为第八版。
  - [Wireshark Labs](http://gaia.cs.umass.edu/kurose_ross/wireshark.htm): 书籍配套的 Wireshark 实验
  - [Instructor Resources](http://gaia.cs.umass.edu/kurose_ross/instructor.htm): 与该书籍配套的一些重要学习指导资源。
  - [Online presentations Video](http://gaia.cs.umass.edu/kurose_ross/online_lectures.htm): 由于新冠病毒的影响，原书作者之一 Jim Kurose 通过在线学习的方式，给学生授课。每章节不仅包含了授课的PPT、视频，还有知识点检测以及授课中问题的汇总，总之，质量是非常的好。
  - [Computer-Networking-A-Top-Down-Approach-8th-Edtion](https://github.com/PEGASUS1993/Computer-Networking-A-Top-Down-Approach-8th-Edtion) Github上一位学习者收集的这本书的一些资料，非常的好。


- 视频
  - B站：[斯坦福大学 Introduction to Computer Networking (CS 144)](https://www.bilibili.com/video/BV137411Z7LR) 


- 他人笔记
  - [知乎：CS144: 什么，你学不会TCP？那就来自己写一个吧！](https://zhuanlan.zhihu.com/p/175998415)  
  - [返回主页康宇PL's Blog](https://www.cnblogs.com/kangyupl/p/stanford_cs144_labs.html) <font color=red>笔记很好，需要深入挖掘。 </font>
- Lab
  - [Github: huangrt01TCP-Lab](https://github.com/huangrt01/TCP-Lab) 
  - [Github: CS144 sponge](https://github.com/cs144/sponge): 重点推荐。


# 7. 数据结构与算法

# 8. 设计模式


# 9. 数据库
# 10. Git-SVN

# 11. Go


# 12. 面试

# 13. Linux
# 14. Shell


# 15. Redis

# 16. Nginx

# 17. 分布式
## 17.1. MIT 6.824
- 视频：
  - B站：[MIT 6.824](https://www.bilibili.com/video/BV1qk4y197bB)
- 书籍：《数据密集型应用系统设计》

# 18. Qt

# 19. 汇编语言

# 20. Markdown


# 21. 哲学
自己本身对哲学十分感兴趣，记下自己平时生活中的一些哲学思想。



如何把明面做好，相当于是做一个宣传，需要好好的的规划。

如何把复杂的东西描绘成最简单的东西？让一个门外汉也能看懂自己写的文章。

学习知识形成体系结构，脑海里有概念，对计算机专业学科的一些专业术语要有了解，这样看英文源文档，才看得懂，看的速度才快。


# 22. 自学指导
- [cs_study_plan](https://github.com/spring2go/cs_study_plan): Github上一份硬核计算机科学CS自学计划
- [TeachYourselfCS-CN](https://github.com/keithnull/TeachYourselfCS-CN/blob/master/TeachYourselfCS-CN.md): 自学计算机科学课程，推荐一些比较好的书籍和课程。
