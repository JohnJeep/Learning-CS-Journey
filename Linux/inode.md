<!--
 * @Author: JohnJeep
 * @Date: 2020-04-04 09:46:51
 * @LastEditTime: 2025-04-04 19:15:49
 * @LastEditors: JohnJeep
 * @Description: iNode的理解
--> 

# 1. 查看iNode的命令
- 查看每个硬盘分区的iNode总数和已经使用的数量 `df -i`
- 查看当前路径下每个iNode的节点号  `ls  -li`
- 查看某个文件的iNode信息  `stat test.txt`


# 2. 什么是iNode?
概念：存储文件的元数据的区域叫iNode，也叫"索引节点"


# 3. iNode包含内容
- 文件的字节数
- 文件的read、write、execute权限
- 文件的ID（User ID、Group ID）
- 文件的时间戳
   - ctime:  iNode上一次变动的时间
   - mtime:  文件内容上一次变动的时间
    - atime:  文件上一次打开的时间
- 链接数： 有多少文件名指向这个iNode
- 文件数据block的位置
- 创建时间


# 4. 操作系统是如何打开文件的？
- 操作系统用iNode号码来识别不同的文件。分为三步：
  1.  系统找到这个文件名对应的iNode号
  2. 通过iNode号得到iNode中存储的信息
  3. 根据iNode中的信息，找到文件数据所在的block区，读出数据


# 5. 目录文件
- Unix/Linux系统中，目录（directory）也是一种文件。打开目录，实际上就是打开目录文件。
- 每个目录项，由两部分组成：所包含文件的文件名，以及该文件名对应的iNode号。
- 根目录`/`的iNode号是固定的


# 6. iNode的应用
1. 有时，文件名包含特殊字符，无法正常删除。这时，直接删除inode节点，就能起到删除文件的作用。
2. 移动文件或重命名文件，只是改变文件名，不影响inode号码。
3. 打开一个文件以后，系统就以inode号码来识别这个文件，不再考虑文件名。因此，通常来说，系统无法从inode号码得知文件名。

- 删除iNode号为 `3453534` 的文件
  - `find . inum 3453534 -delete` 
  - ` find . -inum 3453534 -exec rm -i {} /;`

> 软件更新的机制：在不关闭软件的情况下进行更新，不需要重启。因为系统通过inode号码，识别运行中的文件，不通过文件名。更新的时候，新版文件以同样的文件名，生成一个新的inode，不会影响到运行中的文件。等到下一次运行这个软件的时候，文件名就自动指向新版文件，旧版文件的inode则被回收。


# 7. 硬链接与符号链接
1. 创建硬链接：`ln source  dest_hard_link_file`
2. 创建符号链接：`ln -s  source  dest_symbolic_link_file`

- 硬链接：用不同的文件名可以访问相同的内容，修改文件里面的内容，会影响所有的文件名，但是，删除一个文件名不会影响另一个文件名。
- 软连接（或称符号链接）：相当于一个快捷方式。文件A和文件B的inode号不一样，但是文件A的内容是文件B的路径。读取文件A时，系统会自动将访问者导向文件B。因此，无论打开哪一个文件，最终读取的都是文件B。

  - 删除文件B，则不能打开文件A，因为文件A依赖于文件B而存在。
  - 文件A指向的是文件B的文件名，而不是文件B的iNode号，因此。文件B的iNode连接数不会改变。
  - source和destination都必须指定完整的路径，才创建成功，否则创建的是个文本文件。source必须是已存在的文件，destination是没有创建的文件，需要自己指定。


# 8. 参考
- [linux中inode包含什么内容？](https://mp.weixin.qq.com/s/u9t6QtYCRJAJVpEgxC8t0Q)