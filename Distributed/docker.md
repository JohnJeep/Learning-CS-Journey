- [THE CONTAINER NETWORKING LANDSCAPE: CNI FROM COREOS AND CNM FROM DOCKER](https://thenewstack.io/container-networking-landscape-cni-coreos-cnm-docker) 一篇英文文章，讲解 docker 和 container ecosystem。
- [Github 英文讲解 CNI - the Container Network Interface](https://github.com/containernetworking/cni)

# Docker 常用指令

```
# 查看所有镜像
docker images
# 查看所有运行中的容器
docker ps
# 查看所有容器
docker ps -a
# 获取所有容器ip
$ docker inspect --format='{{.Name}} - {{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' $(docker ps -aq)
# 查看容器内部日志
$ docker logs -f <容器ID>
# 进入容器内部
$ docker exec -it <容器ID> /bin/basj
# 创建容器 -d代表后台启动
docker run --name <容器名称> -e <参数> -v <挂载数据卷> <容器ID>
# 重启容器
docker restart <容器ID>
# 关闭容器
docker stop <容器id>
# 运行容器
docker start <容器id>
```

# Reference

[Docker搭建Zookeeper&Kafka集群](https://www.cnblogs.com/Jacian/p/11421114.html)

