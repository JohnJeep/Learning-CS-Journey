<!--
 * @Author: JohnJeep
 * @Date: 2024-10-10 15:31:51
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-11-20 12:45:11
 * @Description: Git develop flow
 * Copyright (c) 2025 by John Jeep, All Rights Reserved. 
-->

# 开发流程

1. 克隆仓库，拉取最新代码
   ```shell
   cd workspace
   
   git clone <remote_repository_url>
   ```
2. 创建分支并切换到要拉取代码的新分支
   ```shell
   git checkout -b <new_branch_name>
   ```
3. 功能开发，提交commit
   ```shell
   git add 
   git commit
   ```

4. 功能开发完成，先拉取远程仓库最新的代码到本地开发分支，若有冲突，则解决冲突，解决完成后，合并到本地 dev 分支，最后推送到远程的 dev 分支。
   采用 merge 方式：
   ```shell
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
   
   ```shell
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
   

# Git 常用命令

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