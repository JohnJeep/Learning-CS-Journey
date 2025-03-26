<!--
 * @Author: JohnJeep
 * @Date: 2020-09-05 23:42:59
 * @LastEditTime: 2025-03-26 15:42:18
 * @LastEditors: JohnJeep
 * @Description: Nginx learning
 * -->

- [1. Install](#1-install)
  - [1.1. Source code install](#11-source-code-install)
  - [1.2. Binary install](#12-binary-install)
  - [1.3. Docker install](#13-docker-install)
- [2. command](#2-command)
- [3. Configure](#3-configure)
- [4. Nginx Application](#4-nginx-application)
- [5. References](#5-references)



# 1. Install

## 1.1. Source code install

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
```

Nginx 编译参数

| 参数                                     | 描述                                                         |
| ---------------------------------------- | ------------------------------------------------------------ |
| `--prefix=value`                         | 指向安装目录                                                 |
| `--sbin-path`                            | 指向（执行）程序文件（`nginx`）                              |
| `--conf-path=`                           | 指向配置文件（`nginx.conf`）                                 |
| `--error-log-path=`                      | 指向错误日志目录                                             |
| `--pid-path=`                            | 指向 `pid` 文件（`nginx.pid`）                               |
| `--lock-path=`                           | 指向 `lock` 文件（`nginx.lock`）（安装文件锁定，防止安装文件被别人利用，或自己误操作。）进程ID文件 |
| `--user=`                                | 指定程序运行时的用户名                                       |
| `--group=`                               | 指定程序运行时的用户组名                                     |
| `--builddir=`                            | 指向编译目录                                                 |
| `--with-rtsig_module`                    | 启用 rtsig 模块支持（实时信号）                              |
| `--with-select_module`                   | 启用 select 模块支持（一种轮询模式,不推荐在高载环境下使用）禁用：`--without-select_module` |
| `--with-poll_module`                     | 启用 poll 模块支持（功能与 select 相同，与 select 特性相同，为一种轮询模式,不推荐在高载环境下使用） |
| `--with-file-aio`                        | 启用 file aio 支持（一种 APL 文件传输格式）                  |
| `--with-ipv6`                            | 启用 ipv6 支持                                               |
| `--add-module=`                          | 启用外部模块支持                                             |
| `--with-cc=`                             | 指向 C 编译器路径                                            |
| `--with-cpp=`                            | 指向 C 预处理路径                                            |
| `--with-cc-opt=`                         | 设置 C 编译器参数                                            |
| `--with-ld-opt=`                         | 设置连接文件参数                                             |
| `--with-cpu-opt=`                        | 指定编译的 CPU，可用的值为：pentium, pentiumpro, pentium3, pentium4, athlon, opteron, amd64, sparc32, sparc64, ppc64 |
| `--without-pcre`                         | 禁用 PCRE 库                                                 |
| `--with-pcre`                            | 启用 PCRE 库                                                 |
| `--with-pcre=`                           | 指向 PCRE 库文件目录                                         |
| `--with-pcre-opt=`                       | 在编译时为 PCRE 库设置附加参数                               |
| `--with-md5=`                            | 指向 MD5 库文件目录（消息摘要算法第五版，用以提供消息的完整性保护） |
| `--with-md5-opt=`                        | 在编译时为 MD5 库设置附加参数                                |
| `--with-md5-asm`                         | 使用 MD5 汇编源                                              |
| `--with-sha1=`                           | 指向 sha1 库目录（数字签名算法，主要用于数字签名）           |
| `--with-sha1-opt=`                       | 在编译时为 sha1 库设置附加参数                               |
| `--with-sha1-asm`                        | 使用 sha1 汇编源                                             |
| `--with-perl=`                           | 设定 perl 库文件路径                                         |
| `--with-zlib=`                           | 指向 zlib 库目录                                             |
| `--with-zlib-opt=`                       | 在编译时为 zlib 设置附加参数                                 |
| `--with-zlib-asm=`                       | 为指定的 CPU 使用 zlib 汇编源进行优化，CPU 类型为 pentium, pentiumpro |
| `--with-libatomic`                       | 为原子内存的更新操作的实现提供一个架构                       |
| `--with-libatomic=`                      | 指向 libatomic_ops 安装目录                                  |
| `--with-openssl=`                        | 指向 openssl 安装目录                                        |
| `--with-openssl-opt`                     | 在编译时为 openssl 设置附加参数                              |
| `--with-debug`                           | 启用 debug 日志                                              |
| `--with-http_ssl_module`                 | 启用 ngx_http_ssl_module 支持（使支持 HTTPS 请求，需已安装 openssl） |
| `--with-http_realip_module`              | 启用 ngx_http_realip_module 支持（这个模块允许从请求标头更改客户端的 IP 地址值，默认为关） |
| `--with-http_addition_module`            | 启用 ngx_http_addition_module 支持（作为一个输出过滤器，支持不完全缓冲，分部分响应请求） |
| `--with-http_xslt_module`                | 启用 ngx_http_xslt_module 支持（过滤转换 XML 请求）          |
| `--with-http_image_filter_module`        | 启用 ngx_http_image_filter_module 支持（传输 JPEG/GIF/PNG 图片的一个过滤器）（默认为不启用。GD 库要用到） |
| `--with-http_geoip_module`               | 启用 ngx_http_geoip_module 支持（该模块创建基于与 MaxMind GeoIP 二进制文件相配的客户端 IP 地址的 ngx_http_geoip_module 变量） |
| `--with-http_sub_module`                 | 启用 ngx_http_sub_module 支持（允许用一些其他文本替换 Nginx 响应中的一些文本） |
| `--with-http_dav_module`                 | 启用 ngx_http_dav_module 支持（增加 PUT、DELETE、MKCOL 创建集合，COPY 和 MOVE 方法）默认情况下为关闭，需编译开启 |
| `--with-http_flv_module`                 | 启用 ngx_http_flv_module 支持（提供寻求内存使用基于时间的偏移量文件） |
| `--with-http_gzip_static_module`         | 启用 ngx_http_gzip_static_module 支持（在线实时压缩输出数据流） |
| `--with-http_random_index_module`        | 启用 `ngx_http_random_index_module` 支持（从目录中随机挑选一个目录索引） |
| `--with-http_secure_link_module`         | 启用 ngx_http_secure_link_module 支持（计算和检查要求所需的安全链接网址） |
| `--with-http_degradation_module`         | 启用 ngx_http_degradation_module 支持（允许在内存不足的情况下返回204或444码） |
| `--with-http_stub_status_module`         | 启用 ngx_http_stub_status_module 支持（获取 Nginx 自上次启动以来的工作状态） |
| `--without-http_charset_module`          | 禁用 ngx_http_charset_module 支持（重新编码 WEB 页面，但只能是一个方向--服务器端到客户端，并且只有一个字节的编码可以被重新编码） |
| `--without-http_gzip_module`             | 禁用 ngx_http_gzip_module 支持（该模块同 `--with-http_gzip_static_module` 功能一样） |
| `--without-http_ssi_module`              | 禁用 ngx_http_ssi_module 支持（该模块提供了一个在输入端处理处理服务器包含文件（SSI）的过滤器，目前支持 SSI 命令的列表是不完整的） |
| `--without-http_userid_module`           | 禁用 ngx_http_userid_module 支持（该模块用来处理用来确定客户端后续请求的 `cookie` ） |
| `--without-http_access_module`           | 禁用 ngx_http_access_module 支持（该模块提供了一个简单的基于主机的访问控制。允许/拒绝基于 IP 地址） |
| `--without-http_auth_basic_module`       | 禁用 ngx_http_auth_basic_module（该模块是可以使用用户名和密码基于 HTTP 基本认证方法来保护你的站点或其部分内容） |
| `--without-http_autoindex_module`        | 禁用 ngx_http_autoindex_module 支持（该模块用于自动生成目录列表，只在 `ngx_http_index_module` 模块未找到索引文件时发出请求。） |
| `--without-http_geo_module`              | 禁用 ngx_http_geo_module 支持（创建一些变量，其值依赖于客户端的IP地址） |
| `--without-http_map_module`              | 禁用 ngx_http_map_module 支持（使用任意的键/值对设置配置变量） |
| `--without-http_split_clients_module`    | 禁用 ngx_http_split_clients_module 支持（该模块用来基于某些条件划分用户。条件如：ip地址、报头、cookies等等） |
| `--without-http_referer_module`          | 禁用 ngx_http_referer_module支持（该模块用来过滤请求，拒绝报头中 Referer 值不正确的请求） |
| `--without-http_rewrite_module`          | 禁用 ngx_http_rewrite_module ，链接重写                      |
| `--without-http_proxy_module`            | 禁用 ngx_http_proxy_module 支持（有关代理服务器）            |
| `--without-http_fastcgi_module`          | 禁用 ngx_http_fastcgi_module 支持（该模块允许 Nginx 与 FastCGI 进程交互，并通过传递参数来控制 FastCGI 进程工作。 ）FastCGI 一个常驻型的公共网关接口。 |
| `--without-http_uwsgi_module`            | 禁用 ngx_http_uwsgi_module 支持（该模块用来医用uwsgi协议，uWSGI服务器相关） |
| `--without-http_scgi_module`             | 禁用 ngx_http_scgi_module支持                                |
| `--without-http_memcached_module`        | 禁用 ngx_http_memcached_module 支持（该模块用来提供简单的缓存，以提高系统效率） |
| `-without-http_limit_zone_module`        | 禁用 ngx_http_limit_zone_module 支持（该模块可以针对条件，进行会话的并发连接数控制） |
| `--without-http_limit_req_module`        | 禁用 ngx_http_limit_req_module 支持（该模块允许你对于一个地址进行请求数量的限制用一个给定的session或一个特定的事件） |
| `--without-http_empty_gif_module`        | 禁用 `ngx_http_empty_gif_module` 支持（该模块在内存中常驻了一个1*1的透明GIF图像，可以被非常快速的调用） |
| `--without-http_browser_module`          | 禁用 ngx_http_browser_module 支持                            |
| `--without-http_upstream_ip_hash_module` | 禁用 ngx_http_upstream_ip_hash_module 支持（该模块用于简单的负载均衡） |
| `--with-http_perl_module`                | 启用 ngx_http_perl_module 支持（该模块使nginx可以直接使用perl或通过ssi调用perl） |
| `--with-perl_modules_path=`              | 设定 perl 模块路径                                           |
| `--http-log-path=`                       | 设定 access log 路径                                         |
| `--http-client-body-temp-path=`          | 设定 HTTP 客户端请求临时文件路径                             |
| `--http-proxy-temp-path=`                | 设定 HTTP 代理临时文件路径                                   |
| `--http-fastcgi-temp-path=`              | 设定 HTTP Fastcgi 临时文件路径                               |
| `--http-uwsgi-temp-path=`                | 设定 HTTP uwsgi 临时文件路径                                 |
| `--http-scgi-temp-path=`                 | 设定 HTTP scgi 临时文件路径                                  |
| `--without-http`                         | 禁用 HTTP server 功能                                        |
| `--without-http-cache`                   | 禁用 HTTP Cache 功能                                         |
| `--with-mail`                            | 启用 POP3/IMAP4/SMTP 代理模块支持                            |
| `--with-mail_ssl_module`                 | 启用 ngx_mail_ssl_module 支持                                |
| `--without-mail_pop3_module`             | 禁用 POP3 协议                                               |
| `--without-mail_imap_module`             | 禁用 IMAP 协议                                               |
| `--without-mail_smtp_module`             | 禁用 SMTP 协议                                               |
| `--with-google_perftools_module`         | 启用 ngx_google_perftools_module 支持（调试用，剖析程序性能瓶颈） |
| `--with-cpp_test_module`                 | 启用 ngx_cpp_test_module 支持                                |

使用源码安装的 nginx 是没有 `nginx.service` 的，需要手动配置后，才能设置为开机自启。一般 `nginx.service` 位于 `/lib/systemd/system/nginx.service`

`nginx.service`
```shell
# Stop dance for nginx
# =======================
#
# ExecStop sends SIGQUIT (graceful stop) to the nginx process.
# If, after 5s (--retry QUIT/5) nginx is still running, systemd takes control
# and sends SIGTERM (fast shutdown) to the main process.
# After another 5s (TimeoutStopSec=5), and if nginx is alive, systemd sends
# SIGKILL to all the remaining processes in the process group (KillMode=mixed).
#
# nginx signals reference doc:
# http://nginx.org/en/docs/control.html
#
[Unit]
Description=A high performance web server and a reverse proxy server
Documentation=man:nginx(8)
After=network-online.target remote-fs.target nss-lookup.target
Wants=network-online.target

[Service]
Type=forking
PIDFile=/run/nginx.pid
ExecStartPre=/usr/sbin/nginx -t -q -g 'daemon on; master_process on;'
ExecStart=/usr/sbin/nginx -g 'daemon on; master_process on;'
ExecReload=/usr/sbin/nginx -g 'daemon on; master_process on;' -s reload
ExecStop=-/sbin/start-stop-daemon --quiet --stop --retry QUIT/5 --pidfile /run/nginx.pid
TimeoutStopSec=5
KillMode=mixed

[Install]
WantedBy=multi-user.target
```


## 1.2. Binary install

Debian, Ubuntu 及发行版安装
```shell
sudo apt update
sudo apt install neginx
```


## 1.3. Docker install

- 官网镜像：https://hub.docker.com/_/nginx/
- 下载镜像：`docker pull nginx`
- 启动容器
  ```
  docker run --name my-nginx -p 80:80 -v /data/docker/nginx/logs:/var/log/nginx -v /data/docker/nginx/conf/nginx.conf:/etc/nginx/nginx.conf:ro -d nginx
  ```
- 重新加载配置：`docker exec -it my-nginx nginx -s reload`
- 停止服务：`docker exec -it my-nginx nginx -s stop` 或者：`docker stop my-nginx`
- 重新启动服务：`docker restart my-nginx`


# 2. command

```sh
# 停止nginx
nginx -s quit

# 重启nginx，并重新载入配置文件nginx.conf
nginx -s reload

# 重新打开日志文件，一般用于切割日志
nginx -s reopen

# 查看版本
nginx -v

# 查看详细版本信息，包括编译参数
nginx -V

# 检查对配置文件的修改是否正确
nginx -t

# 指定配置文件
nginx -c filename

#强制停止nginx
kill -9 nginx：

# 查看nginx进程
ps -ef | grep nginx 

# 查看nginx的所有进程在TCP、UDP传输中的所有状态
netstat -apn | grep nginx 
```

systemctl 来管理 nginx 进程
```sh
# systemd 开机自启动
systemctl enable nginx

# 停止开机自启
systemctl disable nginx

# 查看设否设置为开机自启
systemctl is-enabled nginx

# 启动
systemctl start nginx

# 查看状态
systemctl status nginx

# 停止
systemctl stop nginx

# 重启
systemctl restart nginx
```

# 3. Configure

nginx 配置文件中的内置变量，也是全局变量
| Variable          | 描述                                               |
| ----------------- | -------------------------------------------------- |
| $args             | 这个变量等于请求行中的参数，同$query_string        |
| $content_length   | 请求头中的Content-length字段                       |
| $content_type     | 请求头中的Content-Type字段                         |
| $document_root    | 当前请求在root指令中指定的值                       |
| $host             | 请求主机头字段，否则为服务器名称                   |
| $http_user_agent  | 客户端agent信息                                    |
| $http_cookie      | 客户端cookie信息                                   |
| $limit_rate       | 限制连接速率                                       |
| $request_method   | 客户端请求的动作，通常为GET或POST                  |
| $remote_addr      | 客户端的IP地址                                     |
| $remote_port      | 客户端的端口                                       |
| $remote_user      | 经过Auth Basic Module验证的用户名                  |
| $request_filename | 当前请求的文件路径，由root或alias指令与URI请求生成 |
| $scheme           | HTTP方法（如http，https）                          |
| $server_protocol  | 请求使用的协议                                     |
| $server_addr      | 服务器地址                                         |
| $server_name      | 服务器名称                                         |
| $server_port      | 请求到达服务器的端口号                             |
| $request_uri      | 包含请求参数的原始URI，不包含主机名                |
| $uri              | 不带请求参数的当前URL                              |
| $document_uri     | 与$uri相同                                         |



# 4. Nginx Application

- Web服务器
- 反向代理
- 负载均衡
- 动态分离
  - 为了加快网站的解析速度，把动态页面和静态页面分别用不同的服务器来解析，加快解析速度，降低原来单个服务器的压力。



# 5. References

- Nginx official document: https://nginx.org/en/docs
- Nginx 极简教程: https://github.com/dunwu/nginx-tutorial
- Tencent Nginx 最全操作总结: https://mp.weixin.qq.com/s/LmtHTOVOvdcnMBuxv7a9_A
- CentOS 7 源码编译安装 Nginx: https://www.cnblogs.com/stulzq/p/9291223.html#top