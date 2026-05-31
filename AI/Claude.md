## Claude 使用技巧

### 小技巧

```txt
/teleport: Resume a Claude Code session from claude.ai

/loop: 让 claude 按照设定的时间间隔自动运行

/schedule:

/resume: 恢复以前的 conversion

/branch: fork 当前 session

/btw: agent 中随时提问，不会打断当前运行的任务

/batch: 并行大规模的变更

/add-dir: 增加一个新的工作目录

/voice: 用语音输入

用 hooks 在 Agent 生命周期中确定性地执行
cowork dispatch: 手机上远程操作电脑

/clear: 清空对话历史，切换任务时避免旧上下文干扰
/compact: 压缩当前对话上下文，释放 token 空间，适合长会话或复杂任务
/init: 扫描当前项目并自动生成 CLAUDE.md 文件，快速建立项目记忆
/memory: 打开交互界面，可以查看和编辑当前的 CLAUDE.md 内容。

/config: 打开配置菜单，调整模型选择、工具权限等设置
/doctor: 对 Claude Code 安装进行健康检查，排查配置或连接问题
/usage: 查看当前 token 消耗，监控费用，适合长时间或高强度会话

/review: 触发代码审查工作流
/help: 显示所有可用命令，包括自定义命令，支持自动补全
/exit: 正确退出当前会话（而非直接关闭终端）


/ : 唤醒命令
@: 添加文件
#: 输入 # 开头的内容，会创建一个 仅限当前会话 的临时指令，加载到当前对话上下文中，但不会写入 CLAUDE.md 文件，也不会持久化到下次会话。

```

claude  -w: 使用 git worktree；
claude -p:  默认的运行方式；
claude --bare ：提升 SDK 启动速度；
claude  --agent：给 claude code指定自定义系统提示和工具 ；


## 模块

### skills



### subagents



### plugins



### MCP


