<!--
 * @Author: JohnJeep
 * @Date: 2026-11-07 16:50:40
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-11-07 17:23:12
 * @Description: Linux install oh-my-zsh and common plugins
 * Copyright (c) 2026 by John Jeep, All Rights Reserved.
-->
# Linux 下安装 oh-my-zsh 与常用插件

## 一、前置准备

查看系统中是否有下面三个软件
```bash
zsh --version
git --version
curl --version
```

若有，则不用安装；若没有，输入下面的操作进行安装。

**macOS**:系统自带 zsh(Catalina 之后默认 shell),只需确认 git:
```bash
zsh --version
git --version   # 没有的话会提示安装 Xcode Command Line Tools
```

**Ubuntu / Debian**:
```bash
sudo apt update
sudo apt install -y zsh git curl
```

**CentOS / RHEL / Fedora**:
```bash
sudo dnf install -y zsh git curl    # 老系统用 yum
```


## 二、一键安装(推荐)

将下面的脚本保存为 `setup-zsh.sh`,执行 `bash setup-zsh.sh`:
```bash
#!/usr/bin/env bash
set -e

# 1. 安装 oh-my-zsh(不自动切换 shell,不自动进入 zsh)
RUNZSH=no CHSH=no sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)"

# 2. 安装第三方插件(统一放在 ~/.oh-my-zsh/custom/plugins/)
git clone --depth=1 https://github.com/zsh-users/zsh-autosuggestions     ~/.oh-my-zsh/custom/plugins/zsh-autosuggestions
git clone --depth=1 https://github.com/zsh-users/zsh-syntax-highlighting ~/.oh-my-zsh/custom/plugins/zsh-syntax-highlighting
git clone --depth=1 https://github.com/zsh-users/zsh-completions         ~/.oh-my-zsh/custom/plugins/zsh-completions

# 3. 启用插件(syntax-highlighting 必须放最后)
sed -i 's/^plugins=(git)$/plugins=(git colored-man-pages history-substring-search zsh-autosuggestions zsh-completions zsh-syntax-highlighting)/' ~/.zshrc

# 4. zsh-completions 需要在 source oh-my-zsh.sh 之前加入 fpath
sed -i '/^source \$ZSH\/oh-my-zsh.sh/i fpath+=$HOME/.oh-my-zsh/custom/plugins/zsh-completions/src' ~/.zshrc

# 5. 追加实用配置
cat >> ~/.zshrc <<'EOF'

# 历史记录
HISTSIZE=50000
SAVEHIST=50000
setopt HIST_IGNORE_ALL_DUPS   # 忽略重复命令
setopt SHARE_HISTORY          # 多终端共享历史

# 自动建议颜色(深色背景下更清晰)
ZSH_AUTOSUGGEST_HIGHLIGHT_STYLE="fg=#888888"

# 上下键按已输入前缀搜索历史
bindkey '^[[A' history-substring-search-up
bindkey '^[[B' history-substring-search-down

# 常用别名
alias reload='source ~/.zshrc'
EOF

echo "安装完成,请执行下一步:切换默认 shell"
```


如果访问 `raw.githubusercontent.com` 失败,先设置代理再执行:
```bash
export https_proxy=http://127.0.0.1:7890
```


## 三、切换到 zsh

设置 zsh 为默认 shell(Linux)

**方式 A:`chsh`(有权限时,不需要 sudo)**
```bash
chsh -s "$(which zsh)"
```

然后**注销并重新登录**生效。


**方式 B:`chsh` 不可用或被禁用时(常见于公司服务器)**
在 `~/.bashrc` 末尾追加,让 bash 登录后自动切到 zsh:

```bash
cat >> ~/.bashrc <<'EOF'

# 交互式 shell 自动切换到 zsh
case $- in
  *i*) command -v zsh >/dev/null 2>&1 && exec zsh ;;
esac
EOF
```

下次登录即生效。限定交互式 shell,不会影响 scp、rsync 等非交互场景。


## 四、手动安装(想了解每一步)

**1. 安装 oh-my-zsh**
```bash
sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)"
```

**2. 克隆三个插件**
```bash
git clone https://github.com/zsh-users/zsh-autosuggestions     ~/.oh-my-zsh/custom/plugins/zsh-autosuggestions
git clone https://github.com/zsh-users/zsh-syntax-highlighting ~/.oh-my-zsh/custom/plugins/zsh-syntax-highlighting
git clone https://github.com/zsh-users/zsh-completions         ~/.oh-my-zsh/custom/plugins/zsh-completions
```

**3. 编辑 `~/.zshrc`**
在 `source $ZSH/oh-my-zsh.sh` 这一行的**上方**加入:
```bash
fpath+=$HOME/.oh-my-zsh/custom/plugins/zsh-completions/src
```

把 `plugins=(git)` 改为:
```bash
plugins=(
  git
  colored-man-pages
  history-substring-search
  zsh-autosuggestions
  zsh-completions
  zsh-syntax-highlighting   # 必须放最后
)
```

**4. 生效**
```bash
source ~/.zshrc
```


## 五、插件功能速览

内置插件

| 插件                       | 作用                                                                    |
| -------------------------- | ----------------------------------------------------------------------- |
| `git`                      | 提供大量 git 别名,如 `gst`(status)、`gco`(checkout)、`gcmsg`(commit -m) |
| `z`                        | 目录跳转,按访问频率智能跳转,如 `z proj`                                 |
| `extract`                  | 万能解压,`x file.tar.gz` 即可,无需记参数                                |
| `sudo`                     | 连按两次 `Esc`,给上一条命令自动加 `sudo`                                |
| `colored-man-pages`        | man 手册彩色显示                                                        |
| `history-substring-search` | 输入部分命令后,用上下键搜索匹配的历史                                   |
| `docker` / `kubectl`       | 对应工具的补全与别名(按需启用)                                          |
| `command-not-found`        | 命令不存在时提示该安装哪个包                                            |


第三方插件

| 插件                      | 作用                               |
| ------------------------- | ---------------------------------- |
| `zsh-autosuggestions`     | 根据历史以灰色显示建议,按 `→` 接受 |
| `zsh-completions`         | 为更多命令提供补全                 |
| `zsh-syntax-highlighting` | 命令正确显示绿色,错误显示红色      |


查看所有 git 别名:
```bash
alias | grep git
```

**Oh-my-zsh 有很多丰富的主题，可以自行去配置，当前教程中使用插件自带的主题。[oh my zsh github themes](https://github.com/ohmyzsh/ohmyzsh/wiki/Themes)**


## 六、统一管理与更新

**更新 oh-my-zsh :**
```bash
omz update
```

**一条命令更新所有第三方插件**(以后新增插件也无需修改命令):
```bash
for d in ~/.oh-my-zsh/custom/plugins/*/; do
  [ -d "$d/.git" ] && echo "更新 $(basename "$d")" && git -C "$d" pull --ff-only
done
```

可以把它做成别名,追加到 `~/.zshrc`:
```bash
alias zsh-update='omz update && for d in ~/.oh-my-zsh/custom/plugins/*/; do [ -d "$d/.git" ] && echo "更新 $(basename "$d")" && git -C "$d" pull --ff-only; done'
```

以后直接运行 `zsh-update` 即可。

**以后新增插件的流程**(统一三步):
```bash
git clone --depth=1 <插件仓库地址> ~/.oh-my-zsh/custom/plugins/<插件名>
# 然后在 ~/.zshrc 的 plugins=(...) 中加入 <插件名>
source ~/.zshrc
```

**卸载:**
```bash
uninstall_oh_my_zsh
```


## 七、常见问题

| 现象                           | 解决办法                                                                                                  |
| ------------------------------ | --------------------------------------------------------------------------------------------------------- |
| 插件不生效                     | 检查 `plugins=()` 里的名字与 `~/.oh-my-zsh/custom/plugins/` 下的目录名是否完全一致,然后 `source ~/.zshrc` |
| 语法高亮不生效                 | 确认 `zsh-syntax-highlighting` 在 plugins 列表**最后一位**                                                |
| 提示 `insecure directories`    | `compaudit                                                                                                | xargs chmod g-w,o-w` |
| 启动变慢                       | `time zsh -i -c exit` 测速,精简插件                                                                       |
| 登录后还是 bash                | 用上面的"方式 B",在 `~/.bashrc` 里 `exec zsh`                                                             |
| `git clone` 超时               | 设置 `https_proxy`,或把 `github.com` 换成可用的镜像地址                                                   |
| 原来 bash 里的 PATH 等配置丢了 | 把 `~/.bashrc` 中的 `export PATH=...` 等语句复制到 `~/.zshrc` 末尾                                        |


## 八、推荐的使用配置

在 `~/.zshrc` 末尾追加:
```bash
# 历史记录
HISTSIZE=50000
SAVEHIST=50000
setopt HIST_IGNORE_ALL_DUPS   # 忽略重复命令
setopt SHARE_HISTORY          # 多个终端共享历史

# 自动建议颜色(默认可能在深色背景下看不清)
ZSH_AUTOSUGGEST_HIGHLIGHT_STYLE="fg=#888888"

# history-substring-search 绑定上下键
bindkey '^[[A' history-substring-search-up
bindkey '^[[B' history-substring-search-down

# 常用别名
alias ll='ls -lFh'
alias ..='cd ..'
alias ...='cd ../..'
alias reload='source ~/.zshrc'
```
