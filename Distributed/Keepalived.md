# Keepalived 

## 安装

命令

```
// 源码配置依赖加载
./configure -prefix=/usr/local/keepalived

// 安装命令
make && make install

// 启动脚本变量引用文件
cp /usr/local/keepalived/etc/sysconfig/keepalived  /etc/sysconfig/

// 将keepalived主程序加入到环境变量
cp /usr/local/keepalived/sbin/keepalived /usr/sbin/

// keepalived启动脚本，放到/etc/init.d/目录下就可以使用service命令便捷调用
cp /你当初解压的keepalived目录/keepalived/etc/init.d/keepalived  /etc/init.d/

// 创建keepalived目录
mkdir /etc/keepalived

// 拷贝keepalived配置文件
cp /usr/local/keepalived/etc/keepalived/keepalived.conf /etc/keepalived/keepalived.conf

// 查看keepalived版本
keepalived --version

// 启动、关闭、重启、查看状态
systemctl start|stop|restart|status keepalived

// 设置开机启动
systemctl enable keepalived
```

## LVS

### 相关术语

```sh
LB (Load Balancer 负载均衡)
HA (High Available 高可用)
Failover (失败切换)
Cluster (集群)
LVS (Linux Virtual Server Linux 虚拟服务器)
DS (Director Server)，指的是前端负载均衡器节点
RS (Real Server)，后端真实的工作服务器
VIP (Virtual IP)，虚拟的 IP 地址，向外部直接面向用户请求，作为用户请求的目标的 IP 地址
DIP (Director IP)，主要用于和内部主机通讯的 IP 地址
RIP (Real Server IP)，后端服务器的 IP 地址
CIP (Client IP)，访问客户端的 IP 地址
```

命令

```
// 实时查看负载均衡状态
watch ipvsadm -Ln --stats
```





## References

- [keepalived centos 安装教程](https://blog.csdn.net/qq_42825214/article/details/105314603)
- [LVS+Keepalived 高可用环境部署记录（主主和主从模式）](https://www.cnblogs.com/kevingrace/p/5574486.html)
- [LVS+KeepAlived高可用部署架构](https://www.cnblogs.com/dooor/p/lvskeepalived0226.html)