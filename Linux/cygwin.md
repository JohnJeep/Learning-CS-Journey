<!--
 * @Author: JohnJeep
 * @Date: 2020-05-24 10:10:40
 * @LastEditTime: 2020-11-16 21:02:15
 * @LastEditors: Please set LastEditors
 * @Description: cygwin使用笔记
--> 

参考：
- [silaoA的博客: Cygwin学习路线](https://silaoa.github.io/2019/2019-06-16-Cygwin%E7%B3%BB%E5%88%97%EF%BC%88%E4%B9%9D%EF%BC%89%EF%BC%9ACygwin%E5%AD%A6%E4%B9%A0%E8%B7%AF%E7%BA%BF.html)：介绍了相关Cygwin的使用，重点值得阅读。
- [Cygwin的使用方法](https://blog.csdn.net/springone/article/details/676667)
- [Cygwin工具使用入门教程](https://www.linuxidc.com/Linux/2019-02/156967.htm)


## 命令
- 查看当前cygwin版本：`cygcheck -c cygwin`
- 显示Windows下的进程 `ps -aW` 
- `iconv` 将文本文件从一种字符编码转换为另一种字符编码  ` iconv -f  输入编码  -t  输出编码  输入文件 > 输出文件`
- `cygcheck` 包管理，查找或者显示包的信息
