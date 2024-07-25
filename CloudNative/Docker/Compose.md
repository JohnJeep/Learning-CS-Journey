<!--
 * @Author: JohnJeep
 * @Date: 2024-01-06 17:19:15
 * @LastEditors: JohnJeep
 * @LastEditTime: 2024-01-06 17:30:46
 * @Description:  Docker Compose 用法
 * Copyright (c) 2024 by John Jeep, All Rights Reserved. 
-->

# 工程开发中常用缩写

| 缩写     | 英文全称                    | 说明                 |
| -------- | --------------------------- | -------------------- |
| dev      | development                 | 开发                 |
| sit      | System Integrate Test       | 系统整合测试（内测） |
| uat      | User Acceptance Test        | 用户验收测试         |
| pet      | Performance Evaluation Test | 性能评估测试（压测） |
| sim      | simulation                  | 仿真                 |
| prd/prod | production                  | 产品/正式/生产       |



## Docker Compose 指令

命令行终端输入 `docker compose --help` 查看 docker compose 的用法。

```sh
[root@dev-tdepoc-ap01 ~]# docker compose --help

Usage:  docker compose [OPTIONS] COMMAND

Define and run multi-container applications with Docker.

Options:
      --ansi string                Control when to print ANSI control characters ("never"|"always"|"auto") (default "auto")
      --compatibility              Run compose in backward compatibility mode
      --dry-run                    Execute command in dry run mode
      --env-file stringArray       Specify an alternate environment file.
  -f, --file stringArray           Compose configuration files
      --parallel int               Control max parallelism, -1 for unlimited (default -1)
      --profile stringArray        Specify a profile to enable
      --progress string            Set type of progress output (auto, tty, plain, quiet) (default "auto")
      --project-directory string   Specify an alternate working directory
                                   (default: the path of the, first specified, Compose file)
  -p, --project-name string        Project name

Commands:
  build       Build or rebuild services
  config      Parse, resolve and render compose file in canonical format
  cp          Copy files/folders between a service container and the local filesystem
  create      Creates containers for a service.
  down        Stop and remove containers, networks
  events      Receive real time events from containers.
  exec        Execute a command in a running container.
  images      List images used by the created containers
  kill        Force stop service containers.
  logs        View output from containers
  ls          List running compose projects
  pause       Pause services
  port        Print the public port for a port binding.
  ps          List containers
  pull        Pull service images
  push        Push service images
  restart     Restart service containers
  rm          Removes stopped service containers
  run         Run a one-off command on a service.
  scale       Scale services
  start       Start services
  stop        Stop services
  top         Display the running processes
  unpause     Unpause services
  up          Create and start containers
  version     Show the Docker Compose version information
  wait        Block until the first service container stops
  watch       Watch build context for service and rebuild/refresh containers when files are updated

Run 'docker compose COMMAND --help' for more information on a command.
```



```sh
# 创建和启动容器，-d 表示：后台运行，detached 简称
docker compose up -d

# 删除实例
docker compose down

# 列出目前正在运行相关容器
docker compose ps

# 启动
docker compose start

# 停止
docker composr stop

# 重启
docker compose restart

# 容器运行时执行输入的命令
# 登录到redis中 -a redis密码
docker-compose exec redis redis-cli -a redis123

# 登录到mysql
docker-compose exec  mysql  mysql -uroot -pMysql@root123
```



# References

- Docker Compose offical doc: https://docs.docker.com/compose/
- Compose 模板文件：https://yeasy.gitbook.io/docker_practice/compose/compose_file