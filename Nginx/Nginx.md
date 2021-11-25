<!--
 * @Author: JohnJeep
 * @Date: 2020-09-05 23:42:59
 * @LastEditTime: 2021-11-26 00:55:59
 * @LastEditors: Windows10
 * @Description: Nginx学习
 * -->

<!-- TOC -->

- [1. 环境安装](#1-环境安装)
- [2. Nginx 常用命令](#2-nginx-常用命令)
- [3. nginx 用途](#3-nginx-用途)
- [4. 参考](#4-参考)

<!-- /TOC -->

# 1. 环境安装

```sh
1. 准备编译环境，自动安装 Nginx 对应的依赖环境
  yum update
  yum -y install gcc pcre pcre-devel zlib zlib-devel openssl openssl-devel
2. 获取源码
  wget http://nginx.org/download/nginx-1.14.0.tar.gz
3. 解压 Nginx 源码
  tar -xzf nginx-1.14.0.tar.gz
4. 进入源码目录
  cd nginx-1.14.0
5. 执行 configure 脚本，设置指定路径
  ./configure --prefix=/opt/nginx1.14  --with-http_ssl_module --with-http_stub_status_module --with-http_realip_module --with-threads
6. 执行 make 命令，调用 gcc 编译工具链
  make
7. 开始安装到指定路径 
  make install
8. 安装成功后，到 Nginx 的 sbin 路径下启动软件，看是否安装成功。
  /opt/nginx1.14/sbin/nginx -V
  
  输出如下：
  nginx version: nginx/1.14.0
  built by gcc 4.8.5 20150623 (Red Hat 4.8.5-28) (GCC) 
  built with OpenSSL 1.0.2k-fips  26 Jan 2017
  TLS SNI support enabled
  configure arguments: --user=www --group=www --prefix=/opt/nginx1.14  --with-http_ssl_module --with-http_stub_status_module --with-threads
9. 手动配置 Nginx 环境变量，终端下任意路径均可启动
  编辑文件：/etc/profile.d/nginx.sh
  写入：export PATH=/opt/nginx1.12/sbin/nginx/sbin
  
10. 退出当前会话，重新登录
   logout
11. 查看环境变量是否失效
  cat /etc/profile/nginx.sh
12. 终端启动 Nginx

13. 设置自开机脚本，并赋予脚本可执行权限
   vim /etc/init.d/nginx.sh
   chmod u+x /etc/init.d/nginx

14. 将nginx服务加入chkconfig管理列表
  chkconfig --add /etc/init.d/nginx
  chkconfig nginx on
  启动: systemctl start nginx
```

参考：[Nginx 配置教程](https://www.cnblogs.com/stulzq/p/9291223.html#top)

# 2. Nginx 常用命令

```sh
nginx -s quit：停止nginx
nginx -s reload：重启nginx，并重新载入配置文件nginx.conf
nginx -s reopen：重新打开日志文件，一般用于切割日志
nginx -v：查看版本
nginx -V：查看详细版本信息，包括编译参数
nginx -t：检查对nginx.conf配置文件的修改是否正确
nginx -c filename：指定配置文件
kill -9 nginx：强制停止nginx
ps -ef | grep nginx 查看nginx进程
netstat -apn | grep nginx： 查看nginx的所有进程在TCP、UDP传输中的所有状态
```

systemctl 来管理 nginx 进程

```sh
sudo systemctl stop nginx： 停止Web服务器
sudo systemctl start nginx：启动Web服务器
sudo systemctl restart nginx：重启web服务器
sudo systemctl reload nginx：重新加载nginx的配置文件
sudo systemctl disable nginx：禁用nginx默认启动
sudo systemctl enable nginx：重新启动nginx并将设置为默认启动
```



# 3. nginx 用途
- Web服务器
- 反向代理
- 负载均衡
- 动态分离
  - 为了加快网站的解析速度，把动态页面和静态页面分别用不同的服务器来解析，加快解析速度，降低原来单个服务器的压力。


# 4. 参考
- [Nginx 极简教程](https://github.com/dunwu/nginx-tutorial)
- [Nginx 入门指南](https://wiki.jikexueyuan.com/project/nginx/)：介绍nginx的是使用
- [Ubuntu安装Nginx](https://www.howtoing.com/how-to-install-nginx-on-ubuntu-18-04)：如何在Ubuntu 18.04上安装Nginx
- [16张图入门Nginx——（前端够用，运维入门）](https://segmentfault.com/a/1190000023648269?utm_source=sf-related)：介绍Nginx的使用
- [高性能 Nginx HTTPS 调优 - 如何为 HTTPS 提速 30%](https://kalasearch.cn/blog/high-performance-nginx-tls-tuning/)：介绍了Nginx调优的技巧。
- [Nginx 最全操作总结](https://mp.weixin.qq.com/s/LmtHTOVOvdcnMBuxv7a9_A)