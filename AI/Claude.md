<!--
 * @Author: JohnJeep
 * @Date: 2026-05-31 10:13:59
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-06-07 13:39:53
 * @Description: Claude Code Using
 * Copyright (c) 2026 by John Jeep, All Rights Reserved. 
-->

## 1. .Claude

ClaudeCode 会将Claude有关的配置都放到 `.claude` 路径下。

让 Claude 明确知道几件事情

- 你的工作。
- 你的沟通风格。
- 你的主要工具。
- 你正在进行的项目。
- 你的硬性约束。

`CLAUDE.md` 分为全局配置、project配置。全局配置位于 `~/.claude/CLAUDE.md`，而工程配置位于 `/project/CLAUDE.md`。

全局配置规则

- 全局的`~/.claude/CLAUDE.md` 最好控制在 200 行以内。超过这个范围后，Claude 对指令的遵守程度会明显下降。如果上下文越来越大，可以用 `.claude/rules/` 目录做模块化规则管理。

- 写入你的语言偏好、技术栈、代码风格规则，以及任何你已经重复解释超过两次的东西。



仓库中配置好的 `.claude` 目录结构如下：

```
.claude/

.claude/
├── CLAUDE.md
├── rules/
│   ├── langgraph.md
│   ├── retrieval.md
│   ├── tests.md
│   └── python-types.md
├── agents/
│   ├── retrieval-reviewer.md
│   ├── prompt-auditor.md
│   └── eval-runner.md
├── skills/
│   ├── new-rag-eval/
│   │   └── SKILL.md
│   └── claude-pr-checklist/
│       └── SKILL.md
├── settings.json
└── .mcp.json
```

项目根目录的 `CLAUDE.md` 会在每次 session 开始时加载。也就是说，它会永久消耗 token。

**准则**

> 不要文件多，要每个文件都很短、很准、很有边界。

1. Memory：短而强制，建议控制在 500 tokens 以下，200 行以内，语气用命令式；
2. 告诉 Memory 要做什么，而不是塞知识库，放真正高频、关键、会影响决策的规则；
3. 复杂的任务，先执行 Plan Mode；
4. 每个 rules 文件只负责某个路径下的行为；
5. 每个 subagent 大概三十行；
6. hooks 只做一个 pre-tool gate 和一个 post-tool formatter
7. MCP server 也不是装几十个，而是只保留几个真正有用的；



### 1.1. memory hierarchy

Claude Code 有一套五层 memory hierarchy:

1. 你的个人偏好;
2. 项目根目录文件;
3. 路径级规则;
4. 本地未提交覆盖;
5. 每个 session 自动写入的 memory；

项目根目录的 `CLAUDE.md` 会在每次 session 开始时加载，会永久消耗 token。

### 1.2. path-scoped rules

处理文件级、目录级的规则。

这种模式通常用 YAML frontmatter。你定义一组 glob paths，只有当 Claude 触碰匹配文件时，规则才会被加载，平时，它不消耗 token

**只在需要时才会被触发加载。**



### 1.3. plan mode

先思考，再动手。

先根据执行的上下文，生成一个明确的计划文档，让你先审阅、修改、确认，然后才进行实施。



## 2. Proxy

对国内用户而言，要使用代理，才能访问 Anthropic 公司的 Claude Code。

### 2.1. WSL2 Proxy


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



### 2.2. ubuntu Container Proxy

VS Code 的 Claude 插件（Claude Code for VS Code）在进行 OAuth 登录时，使用的是 VS Code 自身的网络栈，而不是 WSL2 终端的环境变量。
你在 WSL2 bash 里设置的 HTTPS_PROXY 对 VS Code 图形界面的插件不生效。

打开 VS Code 设置 (Ctrl+,)，搜索 http.proxy，填入：
```bash
# 宿主机 IP，通过 cat /etc/resolv.conf 查看 IP
http://HOST_IP:7897
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



## 3. Plugins

Claude 插件的基石是 **MCP**——这是 Anthropic 在 2025 年 12 月发布的开放标准。它规范了 AI 模型与外部数据及工具的连接方式，终结了以往每个服务都需要定制化集成的繁琐历史。 

插件的四种形态

- MCP Servers：将 Claude 连接至外部服务（如 GitHub、数据库）。
- Skills (技能)：Claude 在特定场景下自动激活的功能。
- Commands (命令)：如 `/code-review` 这样的自定义快捷指令。
- Hooks (钩子)：由事件触发的自动化操作（如强制执行代码规范）。



### 3.1. Feature-Dev

- **功能描述**：通过 7 阶段工作流，将需求直接转化为生产环境代码。
- **核心价值**：大多数 AI 只会“写代码”，而 Feature-Dev 模拟的是**高级工程师的思维模型**。它包含需求拆解、架构探索、设计方案、实现、测试、评审及文档生成全流程。
- **适用场景**：构建完整的功能模块，而非零散的代码片段。
- **安装指令**：`/plugin install feature-dev@claude-plugins-official`



### 3.2. Frontend-Design

- **功能描述**：在动笔写 CSS 之前，赋予 Claude 专业设计师的直觉。
- **核心价值**：它能帮 Claude 摆脱那种廉价的“AI 审美”。插件会让 AI 考虑品牌语调、视觉约束、不对称布局以及滚动触发动画。
- **用户反馈**：“很多人问我设计师是谁，其实只有装了插件的 Claude。”
- **安装指令**：`npx skills add anthropic/frontend-design`



## 4. Commands

常用 CLI:

```
/init: 初始化 session，扫描当前项目并自动生成 CLAUDE.md 文件，快速建立项目记忆；
/clear: 清空对话历史，切换任务时避免旧上下文干扰；
/model opus4.7: 切换 model；
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
claude  -w: w 是 worktree 的简写，使用 git worktree；
claude -p:  默认的运行方式；
claude --bare ：提升 SDK 启动速度；
claude  --agent：给 claude code指定自定义系统提示和工具 ；
```



## 5. Skills

Skills 的作用，是把稳定工作流打包起来。

一个 skill 就是一个文件夹，里面有带 YAML frontmatter 的 markdown 文件。它也可以捆绑 Python 脚本、bash 命令和测试 fixture。

**它的架构依赖 progressive disclosure**：metadata 在 session 启动时加载； 真正的 instructions 只有触发 skill 时才加载； 捆绑资源只有被引用时才加载。

Skills 处理的是 thinking layer，它们告诉 Agent 应该如何理解一个系统，或者应该遵循什么架构模式。



### 5.1. document-skills

用一条命令处理 PDF、XLSX、DOCX 和 PPTX 的生成。

安装方式

```bash
/plugin install document-skills@anthropic-agent-skills
```

如果你没装这个 bundle，却直接让模型“创建一个 PDF”，大概率得到的只是一个顶部写着 PDF 的 markdown 文本。

因为语言模型本质上生成的是文本，不是二进制文件格式。装上这个 bundle 后，Agent 才能真正产出一个 `.pdf` 或 `.xlsx` 文件，让工程以外的人也能打开、阅读和使用。



## 6. MCP

MCP 的全称是 Model Context Protocol，用来把 agent 接到外部工具上。

`mcp-builder` skill 解决的正是本地逻辑和外部状态之间的断层。

Skills 处理的是 thinking layer。它们告诉 Agent 应该如何理解一个系统，或者应该遵循什么架构模式。

MCP servers 处理的是 doing layer。它们负责实时数据、持久状态、OAuth 流程和外部 API 调用。

当你需要连接一个新的内部 billing API 时，`mcp-builder` 可以根据一句自然语言描述，生成所需的 MCP server。



### 6.1. GitHub MCP Server

- **功能描述**：允许 Claude 搜索仓库、阅读代码、开启 Issue 并创建 PR。
- **核心价值**：你不再需要手动复制粘贴代码，直接指令：“分析 src/ 目录并修复 [#42](javascript:;) 号问题”，Claude 会自动完成搜索、阅读、提议并提交 PR。
- **安装方式**：Anthropic 官方插件市场直接启用。



###  6.2. Brave Search MCP

- **功能描述**：将 Claude 连接至**实时互联网**。
- **核心价值**：打破 AI 训练数据的截止日期。无论是调研本周的 AI 动态、查股价还是验证事实，它都能提供最新的合成信息。
- **适用场景**：实时科研、事实核查与新闻汇总。



## 7. Hooks

Hooks 的作用，是让 agent 在更少人工盯着的情况下安全运行。

用 hooks 在 Agent 生命周期中确定性地执行
cowork dispatch: 手机上远程操作电脑



## 8. Subagents

Claude Code 内置的 subagents:

- explore agent 负责只读代码库搜索；
- general-purpose agent 处理需要干净上下文的多步骤任务；
- code-reviewer 和 code-architect 则负责更专门的角色p；













