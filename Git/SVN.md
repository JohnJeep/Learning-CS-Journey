<!--
 * @Author: JohnJeep
 * @Date: 2020-11-20 10:59:05
 * @LastEditTime: 2020-11-20 16:13:24
 * @LastEditors: Please set LastEditors
 * @Description: SVN学习笔记
-->

## 0.1. 参考
- [linux下svn命令使用大全](http://blog.chinaunix.net/uid-22566367-id-1965771.html)

## 0.2. 概念
- 什么是Subversion?
> Subversion的版本库是一种特别的文件版本库，它会记录每一次改变：每个文件的改变，甚至是目
录树本身的改变。

- 文件共享
  - 一般的文件共享采用的是 `lock-modify-unlock` 的方式
  - Subversion、VS和一些版本控制系统使用  `copy-modify-merge`模型。
    > 在这种模型里，每一个客户联系项目版本库建立一个个人工作拷贝—版本库中文件和目录的本地映射。用户并行工作，修改各自的工作拷贝，最终，各个私有的拷贝合并在一起，成为最终的版本，这种系统通常可以辅助合并操作，但是最终要靠人工去确定正误。

-  工作副本
    >  `.svn `为名的文件夹，也被叫做工作副本的管理目录，这个目录里的文件能够帮助 Subversion 识别哪些文件做过修改，哪些文件相对于别人的工作已经过期。


- 修订版本
    > 每当版本库接受了一个提交，文件系统进入了一个新的状态，叫做一次修订`(revision)`，每一个修订版本被赋予一个独一无二的自然数，一个比一个大，初始修订号是0，只创建了一个空目录，没有任何内容。版本库每一次的提交，保存的是工作副本中改变的快照，并不是整个工作副本的快照。
<img src="./figure/svn版本库.png">


- 工作副本怎样跟踪版本库？
  - 工作副本基于的修订版本
  - 时间戳：记录了工作副本从版本库中最后一次的拷贝。   
  -  工作副本中的四中状态
     -   未修改且是当前的。文件在工作目录里没有修改，在工作修订版本之后没有修改提交到版本库。
     -   本地已修改且是当前的。文件在工作目录里已经修改，在工作修订版本之后没有修改提交到版本库。
     -   Unchanged, and out of date。这个文件在工作目录没有修改，但在版本库中已经修了。这个文件最终将更新到最新版本，成为当时的公共修订版本。
     -   Locally changed, and out of date。这个文件在工作目录和版本库都得到修改。一个svn commit将会失败，这个文件必须首先更新，svn update命令会合并公共和本地修改，如果Subversion不可以自动完成，将会让用户解决冲突。


## 0.3. 如何使用subversion满足日常工作中的开发？
1. 拷贝版本库中的项目到本地工作环境中。svn checkout
2. 更新你的工作拷贝。 svn update
3. 做出修改
    - svn add
    - svn delete
    - svn copy
    - svn move
4. 检查修改
   - svn status
   - svn diff 
5. 取消一些修改 。svn revert
6. 解决冲突(合并别人的修改)
   - svn update
   - svn resolved
7. 提交你的修改。svn commit
   



## 0.4. 常用命令
最重要的帮助命令，遇见不会的命令，需要自己查看帮助文档：`svn help subcommand`，其中subcommand为subversion的内建命令。

### 0.4.1. add
-  add： 往版本库中添加新的文件 
 - `svn add *.php` 添加当前目录下所有的php文件

### 0.4.2. checkout
- 将从SVN版本库中拷贝项目到本地工作目录中，得到一个本地拷贝，这个拷贝包括了命令行指定版本库中的HEAD(最新的)版本。
-  例如：`svn checkout svn://192.168.1.131/45dian/brand`


### 0.4.3. commit
- commit：将本地版本库中改变的文件提交到SVN版本库中
- `svn commit -m “LogMessage“ [-N] [--no-unlock] PATH` (如果选择了保持锁，就使用–no-unlock开关)
 - 例如：svn commit -m “add test file for my test“ test.php


### 0.4.4. lock/unlock(加锁/解锁 )
- `svn lock -m “LockMessage“ [--force] PATH`
 - 例如：svn lock -m “lock test file“ test.php


### 0.4.5. update:：更新你的工作拷贝 
 - svn update 会把版本库的修改带到工作拷贝,如果没有给定修订版本,它会把你的工作拷贝更新到 HEAD 修订版本,否则,它会把工作拷贝更新到你用 --revision 指定的修订版本。为了保持同步, svn update 也会删除所有在工作拷贝发现的无效锁定
 - 每个更新的项目开头都有一个表示所做动作的字符
   - A: add
   - B:  Broken lock (third column only)
   - D: delete
   - U: update
   - C: conflict
   - G:  merge
   - E: existed
   - R: replace
   - M: modified，`.svn`目录记录着文件的修改日期和原始内容，



### 0.4.6. status
- `svn status`: 打印所有本地修改的文件，默认情况下，不会联系版本库.
-  `svn status -v`: 显示所有版本控制下的文件。


### 0.4.7. cat
- `svn cat`望检查一个过去的版本而不希望察看它们的区别
```
$ svn cat -r 2 rules.txt
Be kind to others
Freedom = Chocolate Ice Cream
Everything in moderation
Chew with your mouth open
```

### 0.4.8. list
- `svn list`  可以在不下载文件到本地目录的情况下来察看目录中的文件：
```
$ svn list http://svn.collab.net/repos/svn
README
branches/
clients/
tags/
trunk/
```


### 0.4.9. diff
### 0.4.10. copy
### 0.4.11. merge
### 0.4.12. log：查看日志 

