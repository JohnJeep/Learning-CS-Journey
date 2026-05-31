<!--
 * @Author: JohnJeep
 * @Date: 2026-05-31 10:13:59
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-05-31 15:24:26
 * @Description: Claude Code Using
 * Copyright (c) 2026 by John Jeep, All Rights Reserved. 
-->

# Claude


## Proxy


### WSL2 Peoxy


wls2 下 使用 vscode 插件 [claude code for vscode]的 UI 模式，只需打开代理软件 clash-verge，设置 [系统代理]、[虚拟网卡模式] 即可。

wsl2 的 terminal 中使用 claude，设置系统代理，bash 或者 zsh 中添加 proxy 配置：
```bash
# clash-verge 的混合 port 是 7897
export WIN_IP=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}')
export HTTP_PROXY="http://${WIN_IP}:7897"
export HTTPS_PROXY="http://${WIN_IP}:7897"

# 验证是否能访问claude API
#curl -v https://api.anthropic.com/v1/models
```

注：开启 [虚拟网卡模式] 后，不需要另外再 vscode settings 中添加 proxy。



### Ubuntu Container Proxy 

VS Code 的 Claude 插件（Claude Code for VS Code）在进行 OAuth 登录时，使用的是 VS Code 自身的网络栈，而不是 WSL2 终端的环境变量。
你在 WSL2 bash 里设置的 HTTPS_PROXY 对 VS Code 图形界面的插件不生效。

打开 VS Code 设置 (Ctrl+,)，搜索 http.proxy，填入：
```bash
# 宿主机 IP，通过 cat /etc/resolv.conf 查看 IP
http://IP:7897
```

同时也在 .bashrc 或者 .zshrc 中添加对应的代理配置
```bash
# clash-verge 的混合 port 是 7897
export HOST_IP=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}')
export HTTP_PROXY="http://${HOST_IP}:7897"
export HTTPS_PROXY="http://${HOST_IP}:7897"

# 验证是否能访问claude API
#curl -v https://api.anthropic.com/v1/models
```


## Tips

常用 CLI:
```
/init: 初始化 session，扫描当前项目并自动生成 CLAUDE.md 文件，快速建立项目记忆；
/clear: 清空对话历史，切换任务时避免旧上下文干扰；
/model xxx: 切换 model；
/compact: 压缩当前对话上下文，释放 token 空间，适合长会话或复杂任务；
/context: 查看上下文；
/memory: 打开交互界面，可以查看和编辑当前的 CLAUDE.md 内容；
/resume: 恢复以前的 conversion；
/plan: 创建 plan；
/review: 触发代码审查工作流
/help: 显示所有可用命令，包括自定义命令，支持自动补全
/exit: 正确退出当前会话（而非直接关闭终端）
/config: 打开配置菜单，调整模型选择、工具权限等设置
/doctor: 对 Claude Code 安装进行健康检查，排查配置或连接问题
/usage: 查看当前 token 消耗，监控费用，适合长时间或高强度会话

/teleport: Resume a Claude Code session from claude.ai
/loop: 让 claude 按照设定的时间间隔自动运行
/schedule:

/branch: fork 当前 session；
/btw: agent 中随时提问，不会打断当前运行的任务；
/batch: 并行大规模的变更；
/add-dir: 增加一个新的工作目录；
/voice: 用语音输入；
/usage-report: 生成月度分析报告，看自己的时间花在哪里；
/remote-control: 手机上远程控制 电脑；
```


快捷键
```bash
/ : 唤醒命令
@: 添加文件
#: 输入 # 开头的内容，会创建一个 仅限当前会话 的临时指令，加载到当前对话上下文中，但不会写入 CLAUDE.md 文件，也不会持久化到下次会话。
! cmd: session 中直接执行命令行指令
```


命令行 CLI 启动标志
```bash
claude  -w: 使用 git worktree；
claude -p:  默认的运行方式；
claude --bare ：提升 SDK 启动速度；
claude  --agent：给 claude code指定自定义系统提示和工具 ；
```


## 模块

用 hooks 在 Agent 生命周期中确定性地执行
cowork dispatch: 手机上远程操作电脑

### skills



### subagents



### plugins



### MCP


