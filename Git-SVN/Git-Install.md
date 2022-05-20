# Git 安装

Git 是一个开源跨平台的版本管理软件，可以运行在 Windows、Linux/Unix、macOS 等不同的操作系统上。下面讲解在不同平台上的安装。

# Windows

# Linux

电脑连接了网络，安装操作步骤很简单。比如在 Ubuntu 平台下，只需要执行 `sudo apt install git` 就可以；但在 Centos 系统中，使用 yum 源安装的 git 版本是1.7.1，太老了，Github 需要的 Git 版本最低都不能低于1.7.2 。

所以我们一般不用上面的方法。而是下载 git 源码，编译安装，或者因环境的保密性，工作电脑没有连接外网，需要采用源码的形式安装 Git。


源码的安装一般分为三部：配置(configure)、编译(make)、安装(make install)

1. 卸载 CentOS 系统中老的 Git
    ```
    yum remove git
    ```

2. 首先下载 Git 相应的依赖环境，可以在一台有网的电脑中执行下面的命令，将下载好的依赖通过 ftp 工具上传到要安装的服务器中。

   ```
   yum install curl-devel expat-devel gettext-devel openssl-devel zlib-devel gcc perl-ExtUtils-MakeMaker
   ```

3. 去 Git 官网下载所需要的 Git 源码版本，在一台电脑中下载好源码，然后上传至没有网络的 Linux 服务器上。

   官网：https://mirrors.edge.kernel.org/pub/software/scm/git/

4. 解压源码，进入到 git 源码目录

   ```sh
   [root@KF-CFT-AP2 packets]# tar -zxvf git-2.34.1.tar.gz
   [root@KF-CFT-AP2 packets]# cd git-2.34.1
   [root@KF-CFT-AP2 git-2.34.1]#
   ```

5. 检查配置，检测当前操作系统是否有安装 Git 的依赖环境，同时配置 Git 安装路径。

    ```
    [root@KF-CFT-AP2 git-2.34.1]# ./configure --prefix=/usr/local/git
    ```
    若不配置安装路径，操作系统把 git 默认安装到 `/usr/local/bin/` 路径下，配置了安装路径，执行 `./configure` 命令后，设置 git 的安装路径到指定的位置。 

6. 编译

   ```
   [root@KF-CFT-AP2 git-2.34.1]# make
   ```

7. 安装。执行make install 会将 git 安装到第三步指定的 `/usr/local/git` 路径下

   ```
   [root@KF-CFT-AP2 git-2.34.1]# make install
   ```

8. 查看 Git 版本。进入之前指定的安装目录，查看 git 版本，能成功则表示 git 安装完成

   ```sh
   [root@KF-CFT-AP2 git-2.34.1]# cd /usr/local/git/bin
   [root@KF-CFT-AP2 bin]# ls
   git  git-cvsserver  gitk  git-receive-pack  git-shell  git-upload-archive  git-upload-pack
   [root@KF-CFT-AP2 bin]# ./git --version
   git version 2.34.1
   ```

9. 配置环境变量。`/etc/profile` 文件的最后追加 git 的可执行文件的路径 `export PATH=/usr/local/git/bin:$PATH`，修改完成之后，执行 `source /etc/profile` 命令，生效配置文件。若在第五步中没有设置安装路径，则这一步骤可以省略。

   ```
   vi /etc/profile
   ```

10. 在任意的目录下 执行 `git --version`命令，可查看当前 git 安装的版本。

   ```
   [root@KF-CFT-AP2 /]# pwd
   /
   [root@KF-CFT-AP2 /]# git --version
   git version 2.34.1
   ```

   

