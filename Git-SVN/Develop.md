<!--
 * @Author: JohnJeep
 * @Date: 2024-10-10 15:31:51
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-06-05 17:46:37
 * @Description: Git develop flow
 * Copyright (c) 2025 by John Jeep, All Rights Reserved. 
-->

# Develop Workflow

1. 克隆仓库，拉取最新代码
   ```shell
   cd workspace
   
   git clone <remote_repository_url>
   ```
2. 创建分支并切换到要拉取代码的新分支
   ```shell
   git checkout -b <new_branch_name>
   ```
3. 功能开发，提交 commit
   ```shell
   git add 
   git commit
   ```

4. 功能开发完成，先拉取远程仓库最新的代码到本地开发分支，若有冲突，则解决冲突，解决完成后，合并到本地 dev
   分支，最后推送到远程的 dev 分支。
   采用 merge 方式：
   ```bash
   // feature分支下
   git pull origin dev
   
   //本地 Git 库与远程的仓库关联
   git remote add origin git@github.com:liao/learngit.git  
   
   // 第一次推送远程feature
   git push --set-upstream origin feature
   
   // 非第一次推送远程feature
   git push origin feature   // 或直接 git push
   
   git switch dev
   git merge feature
   
   git push origin dev
   ```
   
   采用 rebase 方式：
   
   ```bash
   // feature分支下未做任何的提交
   git pull origin dev
   
   // feature分支下有commit历史
   // 拉取远程最新的提交到本地feature分支下，并将本地之前feature的提交记录放到当前记录的最后
   git pull origin dev --rebase 
   
   // 继续开发
   git add 
   git commit
   
   // 开发完成后，切换到dev分支下，采用merge方式，为了保存合并记录
   git switch dev
   git merge feature
   
   // 本地dev分支下提交到远程dev分支
   git push origin dev
   ```
   

# Git CLI

开发过程中常用的 git 命令。

- `git init` 初始化一个 Git 仓库 。
- `git add <file>` 添加文件到暂存区。
- `git add -p(patch)` 依次存储每一个文件的改动，包括文件中做的哪些改动。
- `git commit -m <message>` 给添加到暂存区的文件增加注释，`-m` 代表是提交信息。
- `git status` 查看当前工作区的状态。
- `git diff` 比较工作区中当前文件和暂存区快照之间的差异。
- `git diff --stage` 查看已暂存的将要添加到下次提交里的内容。
- `git difftool` 使用图像化工具查看工作区与暂存区之间的差异。
- `git reflog` 查看引用日志。每次提交或改变分支都会改变引用日志 `reflog`。
- `git reset --hard HEAD^` 回退到 `HEAD^` 版本。
- `git config --list` 列出 Git 所有当的配置信息。
- `git help <verb>` 查看帮助，verb 为 Git 的关键字。



# Command line instructions

You can also upload existing files from your computer using the instructions below.



## Configure your Git identity

```bash
# local
git config --local user.name "xxx"
git config --local user.email "xxx"

# global
git config --global user.name "xxx"
git config --global user.email "xxx"
```



## Create a new repository

Push files to this repository using SSH or HTTPS. If you're unsure, we recommend SSH.

```
git clone https://xxxx.git
cd your folder
git switch --create main
touch README.md
git add README.md
git commit -m "add README"
git push --set-upstream origin main
```

## Push an existing folder

```bash
# go to your folder
cd existing_folder

# configure the git repository
git init --initial-branch=main
git remote add origin https://xxxx.git
git add .
git commit -m "Initial commit"
git push --set-upstream origin main
```

## Push an existing Git repository

```bash
# go to your repository
cd existing_repo

# configure the git repository
git remote rename origin old-origin
git remote add origin https://xxxx.git
git push --set-upstream origin --all
git push --set-upstream origin --tags
```
