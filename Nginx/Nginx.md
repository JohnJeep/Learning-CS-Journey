<!--
 * @Author: JohnJeep
 * @Date: 2020-09-05 23:42:59
 * @LastEditTime: 2021-01-31 16:10:34
 * @LastEditors: Please set LastEditors
 * @Description: Nginx学习
 * 
-->
## 参考
- [W3Cschool Nginx 入门指南](https://www.w3cschool.cn/nginxsysc/)
- [Nginx 极简教程](https://github.com/dunwu/nginx-tutorial)
- [Nginx 入门指南](https://wiki.jikexueyuan.com/project/nginx/)：介绍nginx的是使用
- [Ubuntu安装Nginx](https://www.howtoing.com/how-to-install-nginx-on-ubuntu-18-04)：如何在Ubuntu 18.04上安装Nginx
- [16张图入门Nginx——（前端够用，运维入门）](https://segmentfault.com/a/1190000023648269?utm_source=sf-related)：介绍Nginx的使用
- [高性能 Nginx HTTPS 调优 - 如何为 HTTPS 提速 30%](https://kalasearch.cn/blog/high-performance-nginx-tls-tuning/)：介绍了Nginx调优的技巧。


## Nginx常用命令
- `nginx -s quit`：停止nginx
- `nginx -s reload`：重启nginx，并重新载入配置文件nginx.conf
- `nginx -s reopen`：重新打开日志文件，一般用于切割日志
- `nginx -v`：查看版本
- `nginx -V`：查看详细版本信息，包括编译参数
- `nginx -t`：检查对nginx.conf配置文件的修改是否正确
- `nginx -c filename`：指定配置文件
- `kill -9 nginx`：强制停止nginx
- `ps -ef | grep nginx` 查看nginx进程
- `netstat -apn | grep nginx`： 查看nginx的所有进程在TCP、UDP传输中的所有状态

systemctl 来管理nginx进程
- `sudo systemctl stop nginx`： 停止Web服务器
- `sudo systemctl start nginx`：启动Web服务器
- `sudo systemctl restart nginx`：重启web服务器
- `sudo systemctl reload nginx`：重新加载nginx的配置文件
- `sudo systemctl disable nginx`：禁用nginx默认启动
- `sudo systemctl enable nginx`：重新启动nginx并将设置为默认启动



## nginx用途
- Web服务器
- 反向代理
- 负载均衡
- 动态分离
  - 为了加快网站的解析速度，把动态页面和静态页面分别用不同的服务器来解析，加快解析速度，降低原来单个服务器的压力。