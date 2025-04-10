<!--
 * @Author: JohnJeep
 * @Date: 2024-12-18 16:25:50
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-04-10 18:21:14
 * @Description: Ollama 学习
 * Copyright (c) 2025 by John Jeep, All Rights Reserved. 
-->



# Introduction

本地启动并运行大型语言模型。
Ollama 是一个开源的大型语言模型服务工具，旨在帮助用户快速在本地运行大模型。通过简单的安装指令，用户可以通过一条命令轻松启动和运行开源的大型语言模型。 它提供了一个简洁易用的命令行界面和服务器，专为构建大型语言模型应用而设计。用户可以轻松下载、运行和管理各种开源 LLM。与传统 LLM 需要复杂配置和强大硬件不同，Ollama 能够让用户在消费级的 PC 上体验 LLM 的强大功能。
Ollama 会自动监测本地计算资源，如有 GPU 的条件，会优先使用 GPU 的资源，同时模型的推理速度也更快。如果没有 GPU 条件，直接使用 CPU 资源。
Ollama 极大地简化了在 Docker 容器中部署和管理大型语言模型的过程，使用户能够迅速在本地启动和运行这些模型。



# Thinking

1. Ollama 后台是怎样启动的？使用了哪些技术？
   - HTTP 建立连接
   - 心跳
2. 工程中 Gin 是怎么使用的？
3. 命令启动时，有点类似Docker的玩法，是怎样实现的？
4. Open-webUI 是怎样跑起来的，使用的是JavaScript，怎样写成网站的？
5. 如何把多台电脑的CPU、GPU、RAM结合在一起跑大模型。
6. 有数亿的人访问大模型，数万亿级别的请求，大模型要做成分布式的，提高并发速度。
7. 部署安全策略，防止病毒或非法入侵。


## 项目结构

仓库包含多个目录和文件，以下是一些主要目录及其功能：
- **app**：应用程序相关代码，包含不同操作系统下的存储路径配置（如 `store_darwin.go`、`store_windows.go`、`store_linux.go`）。
- **llama**：与 Llama 模型相关的代码，如模型加载、上下文处理、KV 缓存等。
- **kvcache**：键值缓存相关代码，定义了缓存接口和包装器。
- **ml**：机器学习后端相关代码，包含 GGML 和 Metal 等后端的实现。
- **server**：服务器相关代码，包括缓存、命令处理等。
- **runner**：运行器相关代码，如 Llama 运行器的缓存实现。


# References

- Ollama 官网: https://ollama.com/

- Ollama Github: https://github.com/ollama/ollama

- 动手学 Ollama 教程: https://github.com/datawhalechina/handy-ollama/blob/main/README.md

- 什么是 ollama: https://wiki.eryajf.net/pages/97047e/

- An open-source, modern-design ChatGPT/LLMs UI/Framework : https://github.com/lobehub/lobe-chat

- open-webui无法链接ollama 报错ERROR:apps.ollama.main:Connection error: Cannot connect: https://www.cnblogs.com/qumogu/p/18235298

  ```
  问题：open-webui正常可以访问，但不能选择Ollama中的模型，解决方法：
  
  # WebUI与ollama在同一台机器：
  sudo docker run -d -p 3000:8080 --add-host=host.docker.internal:host-gateway -v open-webui:/app/backend/data --name open-webui --restart always ghcr.io/open-webui/open-webui:main
  
  # WebUI与ollama不在同一台机器：
  docker run -d -p 3000:8080 -e OLLAMA_BASE_URL=https://example.com -v open-webui:/app/backend/data --name open-webui --restart always ghcr.io/open-webui/open-webui:main
  ```
