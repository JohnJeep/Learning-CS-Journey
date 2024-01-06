<!--
 * @Author: JohnJeep
 * @Date: 2024-01-06 17:19:15
 * @LastEditors: JohnJeep
 * @LastEditTime: 2024-01-06 17:30:46
 * @Description:  Docker Compose 用法
 * Copyright (c) 2024 by John Jeep, All Rights Reserved. 
-->


## Docker Compose 指令

```sh
// 创建实例后，我们可以用 docker-compose stop / docker-compose start 启动和暂停实例

// 创建一个运行实例
docker-compose up -d

// 删除实例
docker-compose down

// 列出目前正在运行相关容器服务
docker-compose ps

// 启动
docker-compose start

// 暂时
docker-composr stop

// 重启
docker-compose restart
```



# References

- Compose 模板文件：https://yeasy.gitbook.io/docker_practice/compose/compose_file