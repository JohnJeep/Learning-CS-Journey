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

   