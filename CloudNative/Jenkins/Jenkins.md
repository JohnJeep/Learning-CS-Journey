<!--
 * @Author: JohnJeep
 * @Date: 2024-01-06 15:19:42
 * @LastEditors: JohnJeep
 * @LastEditTime: 2024-01-06 17:29:55
 * @Description:  Jenkins 学习
 * Copyright (c) 2024 by John Jeep, All Rights Reserved. 
-->


# Jenkins 

Jenkins 是一款开源 CI&CD 软件，用于自动化各种任务，包括构建、测试和部署软件。

![](../figures/jenkins-workflow.png)

## CI/CD

持续集成、自动部署流程

![](../figures/ci-cd.png)

开发人员将代码 push 到 gitlab 中，触发 jenkins 的自动 pull 拉取代码，通过 maven 编译、打包，然后通过执行 shell 脚本使
docker 构建镜像并 push 到私服（或者阿里云）仓库，此操作完成后
jenkins 服务器上再执行 SSH 命令登录到部署服务器，docker 从仓库（私服）拉取镜像，启动容器。整个操作流程完成。



# References
- 【Linux】Docker 搭建 Jenkins: https://open.alipay.com/portal/forum/post/125401045
- https://www.cnblogs.com/kevinwan/p/16007379.html
- 微服务从代码到 k8s 部署应有尽有大结局(k8s 部署)：https://www.cnblogs.com/kevinwan/p/16007379.html

