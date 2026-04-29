<!--
 * @Author: JohnJeep
 * @Date: 2026-01-04 09:53:24
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-04-29 17:53:04
 * @Description: Tmux Usage
 * Copyright (c) 2026 by John Jeep, All Rights Reserved. 
-->

# Tmux

Tmux 是一个终端复用器，允许用户在一个终端窗口中创建、管理和切换多个会话、窗口和面板。

它提供了强大的功能，使得在命令行环境中进行多任务处理变得更加高效和便捷。

Tmux 的主要功能包括：
- 会话(session)管理：用户可以创建多个 session，每个 session 可以包含多个 window 和 pane。
- 窗口(window)管理：每个 session 可以包含多个 window，每个 window 可以运行不同的命令或程序。
- 面板(pane)管理：用户可以将 window 分割成多个 pane，每个 pane 可以独立运行命令或程序。
- 会话恢复(session restore)：用户可以在断开连接后重新连接到之前的 session，继续工作。
- 键绑定(key bindings)：Tmux 提供了丰富的键绑定，允许用户快速执行各种操作。


# tmux sessions

Tmux sessions 是 tmux 的基本单位，每个 session 可以包含多个 window 和 pane。用户可以创建、切换和管理多个 session。

Tmux 创建 sessions 命令： 
```bash
tmux                    # 默认创建一个 session
tmux new -s sessionname # 创建一个名为 sessionname 的 session
```

进入已经存在的 session：
```bash
tmux a                 # attach to the last session
tmux attach-session    # attach to the last session
tmux a -t session-name # attach to a specific session
```

移除 sessions：
```bash
tmux kill-ses                     # 移除当前 session
tmux kill-session -t session-name # 移除指定 session
```

Tmux sessions 按键绑定：
| **_ key bindings** | description      |
| ------------------ | ---------------- |
| CTRL + B $         | rename session   |
| CTRL + B d         | detach session   |
| CTRL + B )         | next session     |
| CTRL + B (         | previous session |


# tmux windows

Tmux windows 可以通过创建新的窗口、切换窗口、重命名窗口等操作来管理。

Tmux windows 按键绑定：
| **_ key bindings** | description              |
| ------------------ | ------------------------ |
| CTRL + B C         | create window            |
| CTRL + B N         | move to next window      |
| CTRL + B P         | move to previous window  |
| CTRL + B L         | move to window last used |
| CTRL + B 0 … 9     | select window by number  |
| CTRL + B ‘         | select window by name    |
| CTRL + B .         | change window number     |
| CTRL + B ,         | rename window            |
| CTRL + B F         | search windows           |
| CTRL + B &         | kill current window      |
| CTRL + B W         | list windows             |


```bash
# list all sessions
tmux ls

# list windows in a specific session:
tmux list-windows -t my_session

#kill a specific window in a session:
tmux kill-window -t my_session:my_window

# delete window number 2 in session my_session
tmux kill-window -t my_session:2
```


# tmux panes

Tmux panes 可以通过水平或垂直分割窗口来创建，每个 pane 都可以独立运行命令或程序。

Tmux panes 按键绑定：
| **_ key bindings** | description                  |
| ------------------ | ---------------------------- |
| CTRL + B %         | vertical split               |
| CTRL + B “         | horizontal split             |
| CTRL + B →         | move to pane to the right    |
| CTRL + B ←         | move to pane to the left     |
| CTRL + B ↑         | move up to pane              |
| CTRL + B ↓         | move down to pane            |
| CTRL + B O         | go to next pane              |
| CTRL + B ;         | go to last active pane       |
| CTRL + B }         | move pane right              |
| CTRL + B {         | move pane left               |
| CTRL + B !         | convent pane to window       |
| CTRL + B X         | kill pane                    |
| CTRL + B q 1 … 9   | Switch/select pane by number |



# tmux copy mode

Tmux copy mode 允许用户在 tmux 中进入一个特殊的模式，在该模式下可以使用键盘进行文本选择、复制和粘贴操作。

copy mode 中的按键绑定：
| **_ key bindings** | description          |
| ------------------ | -------------------- |
| CTRL B [           | enter copy mode      |
| CTRL B ]           | paste from buffer    |
| CTRL L             | clean buffer on pane |
| space              | start selection    |
| enter              | copy selection     |
| esc                | clear selection    |
| g                  | go to top          |
| G                  | go to bottom       |
| h                  | move cursor left   |
| j                  | move cursor down   |
| k                  | move cursor up     |
| l                  | move cursor right  |
| /                  | search             |
| #                  | list paste buffers |
| q                  | quit               |


# config

tmux 的配置文件为 `~/.tmux.conf`

常用设置
```bash
 # 开启鼠标支持
set -g mouse on

# 修改按键：前缀改成ctrl + s
set -g prefix C-s — ctrl + b

# 启用 vi 风格的按键（用于复制模式）
set -g mode-keys vi

# 清当前屏幕和滚动历史
clear && tmux clear-history
```


修改配置文件后使之生效

```bash
# 在 tmux 外部，执行下面的命令
tmux source-file .tmux.conf

# 在 tmux 内
先按 ctrl b : 再执行
source-file .tmux.conf
```

# tmux plugins

Tmux 常用插件：
- Tmux Plugin Manager (TPM)：一个插件管理器，方便用户安装和管理各种 tmux 插件。
- Tmux Resurrect：它能保存会话、窗口、面板布局、工作目录，甚至还能通过扩展保存正在运行的程序（如 vim 的编辑状态）。
- Tmux Yank：一个插件，允许用户在 tmux 中使用系统剪贴板(system clipboard)进行 copy 和 paste 操作。

Tmux Resurrect 插件安装：
```bash
# 克隆 Tmux Resurrect 插件到 TPM 插件目录
git clone https://github.com/tmux-plugins/tmux-resurrect ~/.tmux/plugins/tmux-resurrect


# 在 ~/.tmux.conf 中添加以下行以启用 Tmux Resurrect 插件
set -g @plugin 'tmux-plugins/tmux-resurrect'
run-shell ~/.tmux/plugins/tmux-resurrect/resurrect.tmux

# tmux 外，重新加载 tmux 配置文件
tmux source-file ~/.tmux.conf
```

Tmux Yank 插件安装：
```bash
# 克隆 Tmux Yank 插件到 TPM 插件目录
git clone https://github.com/tmux-plugins/tmux-yank ~/.tmux/plugins/tmux-yank

# 在 ~/.tmux.conf 中添加以下行以启用 Tmux Yank 插件
set -g @plugin 'tmux-plugins/tmux-yank'
run-shell ~/.tmux/plugins/tmux-yank/yank.tmux

# tmux 外，重新加载 tmux 配置文件
tmux source-file ~/.tmux.conf
```


# References

1. Github offical document: https://github.com/tmux/tmux/wiki/Getting-Started
2. https://www.pluralsight.com/resources/blog/cloud/tmux-cheat-sheet
3. https://tmuxcheatsheet.com/
4. https://docs.hpc.sjtu.edu.cn/login/tmux.html
5. https://github.com/tmux-plugins/tmux-yank
6. https://github.com/tmux-plugins/tmux-resurrect
