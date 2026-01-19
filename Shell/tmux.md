## Tmux cheat sheet

### tmux sessions

| **_ new sessions**      |
| ----------------------- |
| tmux                    |
| tmux new                |
| tmux new-session        |
| tmux new -s sessionname |

| **_ attach sessions**  |
| ---------------------- |
| tmux a                 |
| tmux att               |
| tmux attach            |
| tmux attach-session    |
| tmux a -t session-name |

| **_ remove sessions**             |
| --------------------------------- |
| tmux kill-ses                     |
| tmux kill-session -t session-name |

| **_ key bindings** | description      |
| ------------------ | ---------------- |
| CTRL + B $         | rename session   |
| CTRL + B d         | detach session   |
| CTRL + B )         | next session     |
| CTRL + B (         | previous session |

### tmux windows

**_ windows** are like tabs in a browser. Windows exist in sessions and occupy the space of a session screen.

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
#To list windows in a specific session:
tmux list-windows -t my_session

#kill a specific window in a session:
tmux kill-window -t my_session:my_window

# delete window number 2 in session my_session
tmux kill-window -t my_session:2
```


### tmux panes

**_ panes** are sections of windows that have been split into different screens — just like the panes of a real window!

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

### tmux copy mode

| **_ key bindings** | description          |
| ------------------ | -------------------- |
| CTRL B [           | enter copy mode      |
| CTRL B ]           | paste from buffer    |
| CTRL L             | clean buffer on pane |

清当前屏幕和滚动历史

```bash
clear && tmux clear-history
```



| **_ copy mode commands** | description        |
| ------------------------ | ------------------ |
| space                    | start selection    |
| enter                    | copy selection     |
| esc                      | clear selection    |
| g                        | go to top          |
| G                        | go to bottom       |
| h                        | move cursor left   |
| j                        | move cursor down   |
| k                        | move cursor up     |
| l                        | move cursor right  |
| /                        | search             |
| #                        | list paste buffers |
| q                        | quit               |


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

bind -n MouseDown3Pane select-pane
bind -T copy-mode MouseDown3Pane select-pan
```

修改配置文件后使之生效

在 tmux 外部，执行下面的命令

```bash
tmux source-file .tmux.conf
```

在 tmux 内

```bash
先按 ctrl b : 再执行
source-file .tmux.conf
```

## References

1. Github offical document: https://github.com/tmux/tmux/wiki/Getting-Started
2. https://www.pluralsight.com/resources/blog/cloud/tmux-cheat-sheet
3. https://tmuxcheatsheet.com/
4. https://docs.hpc.sjtu.edu.cn/login/tmux.html