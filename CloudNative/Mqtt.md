## EMQX 开放防火墙端口

EMQX 是一个高性能的 MQTT 消息中间件，通常用于物联网和实时消息应用。为了确保 EMQX 可以正常运行并允许客户端连接，你需要在防火墙上开放一些特定的端口。

### 主要端口

1. **MQTT 协议端口**
   - **默认端口：`1883` (TCP)**：这是 MQTT 协议的默认端口，用于客户端与服务器之间的非加密通信。
   - **加密 MQTT（MQTT over TLS/SSL）：`8883` (TCP)**：如果你使用加密的 MQTT（通过 TLS/SSL），需要开放此端口。
2. **MQTT WebSocket 协议端口**
   - **默认端口：`8083` (TCP)**：用于 WebSocket 连接的 MQTT 通信（未加密）。
   - **加密 WebSocket：`8084` (TCP)**：用于加密的 WebSocket 连接（MQTT over WebSocket with TLS/SSL）。
3. **管理界面端口**
   - **默认端口：`18083` (TCP)**：这是 EMQX 的管理控制台端口，通过浏览器访问 Web UI 进行管理和配置。
4. **用于集群通信的端口**
   - **`4371`** (TCP)：用于 Erlang 节点间通信。
   - **`5371`** (TCP)：备用端口用于集群通信。
5. **CoAP 协议端口（可选）**
   - **默认端口：`5683` (UDP)**：如果你启用了 CoAP 协议（用于 IoT 设备），需要开放此端口。
6. **HTTP API 端口（可选）**
   - **默认端口：`8080` (TCP)**：用于 EMQX HTTP API 的端口，如果你使用 EMQX 提供的 REST API 进行集成，可以开放此端口。
7. **用于 MQTT Bridge 的端口（可选）**
   - **默认端口：`1883` 或 `8883` (TCP)**：如果你配置了 MQTT Bridge（用于连接不同的 EMQX 实例或第三方 MQTT 服务器），你需要确保桥接端口也被开放。

### 开放防火墙端口

假设你在 CentOS 或 RHEL 上使用 `firewalld` 作为防火墙工具，可以通过以下命令开放这些端口：

```
# 开放 MQTT 协议端口
sudo firewall-cmd --zone=public --add-port=1883/tcp --permanent
sudo firewall-cmd --zone=public --add-port=8883/tcp --permanent

# 开放 MQTT WebSocket 协议端口
sudo firewall-cmd --zone=public --add-port=8083/tcp --permanent
sudo firewall-cmd --zone=public --add-port=8084/tcp --permanent

# 开放管理界面端口
sudo firewall-cmd --zone=public --add-port=18083/tcp --permanent

# 开放集群通信端口
sudo firewall-cmd --zone=public --add-port=4371/tcp --permanent
sudo firewall-cmd --zone=public --add-port=5371/tcp --permanent

# 开放 CoAP 协议端口（可选）
sudo firewall-cmd --zone=public --add-port=5683/udp --permanent

# 开放 HTTP API 端口（可选）
sudo firewall-cmd --zone=public --add-port=8080/tcp --permanent

# 重新加载防火墙规则
sudo firewall-cmd --reload
```

### 检查防火墙端口是否开放

你可以使用以下命令检查防火墙端口是否已经开放：

```bash
firewall-cmd --list-ports
```

### 总结

对于 EMQX，你至少需要开放以下端口：

- `1883`（非加密 MQTT）
- `8883`（加密 MQTT）
- `8083`（WebSocket MQTT）
- `8084`（加密 WebSocket MQTT）
- `18083`（EMQX 管理控制台）
- `4371` 和 `5371`（集群通信）

根据你的部署和需求，你可能还需要开放其他端口（如 CoAP、HTTP API 等）。



## EMQX 添加环境变量

将 EMQX 添加到系统的环境变量中，可以让你在任何位置执行 EMQX 的命令，而不需要每次都进入其安装目录。以下是将 EMQX 添加到环境变量的步骤。

### 1. 查找 EMQX 安装目录

首先，确认你的 EMQX 安装目录。假设你是按照标准方式安装的 EMQX，那么默认情况下，EMQX 应该安装在 `/opt/emqx` 或 `/usr/local/emqx` 目录下。

如果你不确定安装路径，可以通过以下命令查找 EMQX 的路径：

```bash
whereis emqx
```

或者，如果你是通过源码安装的，可以根据安装时指定的路径来确认。

### 2. 编辑环境变量

假设你的 EMQX 安装目录为 `/opt/emqx`，并且你想将其添加到环境变量中，使得你可以在任何地方通过命令行启动 EMQX。

1. **打开 shell 配置文件**：

   根据你使用的 shell 类型（`bash`、`zsh` 等），编辑相应的配置文件。假设你使用的是 `bash`，配置文件通常是 `~/.bashrc` 或 `/etc/bash.bashrc`（对于系统级别的环境变量）。

   打开 `~/.bashrc` 文件：

   ```bash
   vim ~/.bashrc
   ```

   或者使用 `vim` 编辑器：

   ```bash
   vim ~/.bashrc
   ```

2. **添加 EMQX 的路径到 `PATH`**：

   在 `.bashrc` 文件的末尾添加 EMQX 的路径。假设 EMQX 的安装目录为 `/opt/emqx`，你可以添加如下内容：

   ```bash
   export PATH=$PATH:/opt/emqx/bin
   ```

   这样就将 EMQX 的 `bin` 目录添加到了环境变量中。`/opt/emqx/bin` 目录包含了 EMQX 的可执行文件，例如 `emqx` 命令。

3. **使改动生效**：

   修改完成后，保存并退出编辑器。如果你是用 `nano` 编辑的，可以按 `Ctrl+X` 保存并退出。

   使环境变量更新生效，执行以下命令：

   ```bash
   source ~/.bashrc
   ```

   或者，你也可以关闭当前终端并重新打开一个新的终端窗口。

### 3. 验证是否生效

现在，你可以在任何地方通过以下命令启动 EMQX 或检查其版本，来验证是否已经成功添加到环境变量：

```bash
emqx version
```

如果命令返回 EMQX 的版本号，说明环境变量已经成功配置。

### 4. 系统级环境变量（可选）

如果你想要让所有用户都能够访问 EMQX 命令，你可以将路径添加到系统级环境变量中。对于所有用户，可以编辑 `/etc/profile` 或 `/etc/bash.bashrc` 文件：

```bash
sudo nano /etc/profile
```

在文件末尾添加：

```bash
export PATH=$PATH:/opt/emqx/bin
```

然后执行：

```bash
source /etc/profile
```

或者重启系统。



### 总结

- 确定 EMQX 的安装目录。
- 将 EMQX 的 `bin` 目录路径添加到 `~/.bashrc` 文件中的 `PATH` 环境变量。
- 使更改生效，并验证是否可以通过命令行访问 EMQX。

这样，你就可以方便地在任何地方通过命令行启动 EMQX，而无需进入其安装目录。





## 客户端ID（ClientID）

ClientID 是MQTT连接的唯一标识符。在IoTCore中，不同实例之间是隔离的，故实际上 CoreID + ClientID 唯一标识了一个客户端连接。

如果多个客户端使用相同的 ClientID 连接到MQTT服务器，那么只有最后一个连接会被保留，其他连接会被强制断开。

在使用MQTT连接时，客户端ID应该是唯一的，并且应该易于识别。一个**通常的做法**是，使用固定前缀加上随机生成的UUID作为ClientID：前缀可以用来标识客户端的身份或分组，UUID保证了唯一性。

### 参数设置

在 MQTT 协议中，`retain` 和 `retainHandling` 是两个不同的概念，分别与消息保留和消息传递策略有关。

### 1. `retain` 参数：

- **作用**：决定发布的消息是否被保留在 MQTT 服务器上。
- **范围**：仅在发布消息时有效。
- 取值：true 或  false
  - **`retain = true`**：消息被保留，新的订阅者会立即收到此消息。
  - **`retain = false`**：消息不被保留，新的订阅者不会收到过去发布的消息。

### 2. `retainHandling` 参数：

- **作用**：控制 MQTT 客户端在订阅时如何处理保留消息。
- **范围**：仅在订阅消息时有效。
- 取值 ：`retainHandling`是 MQTT v5 引入的新特性，取值有三种：
  1. **`0`（默认）**：当客户端订阅主题时，接收服务器上的保留消息。
  2. **`1`**：当客户端订阅主题时，**如果客户端以前没有订阅过该主题**，才接收保留消息。
  3. **`2`**：客户端**永远不会**接收保留消息。

### 区别总结：

- **`retain`** 是在发布消息时设置的，用于决定服务器是否应保留该消息。
- **`retainHandling`** 是在订阅时设置的，用于控制客户端是否接收服务器上已经保留的消息。

例如，在某些场景中，用户可能希望只接收新的消息（不包括以前保留的消息），那么可以在订阅时将 `retainHandling` 设置为 `2`。

# TroubleShoots

## 冲突的 MQTT 客户端 ID

- 问题描述

  多个设备使用同一客户端 ID 连接，导致每次有新的数据发送时，原来的连接被断开后重连。

- 分析：

  多个设备使用同一客户端 ID 连接，使用同一客户端 ID 建立了多个连接，从而导致已连接的设备断开连接。MQTT 规范只允许每个客户端 ID 有一个活动连接，因此当另一个设备使用同一客户端 ID 连接时，它会使前一个连接断开。

- 导致的问题：

  ID 相冲突的设备将被迫不断重新连接，这可能导致消息丢失或致使设备无法连接。

  这可能表示设备或设备的凭证已遭破坏，并可能是 DDoS 攻击的一部分。也有可能是设备未在账户中得到正确配置，或者设备连接效果不佳，被迫每分钟重新连接多次。

- 解决方式：

  1. MQTT 连接设备时使用 UUID 作为客户端 ID。
  2. 将每个设备注册为唯一的 客户端ID。

# References

- [centos7设置emqx开机自启动](http://iotts.com.cn/blog/2023/06/28/centos7%E8%AE%BE%E7%BD%AEemqx%E5%BC%80%E6%9C%BA%E8%87%AA%E5%90%AF%E5%8A%A8/)
- https://bifromq.io/zh-Hans/docs/2.0.0/best_practices/mqtt_parameters/