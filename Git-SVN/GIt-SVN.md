服务器使用的是 SVN，但是想本地使用 Git 的 local branch 或者离线编辑代码等，这时 `Git-SVN` 就是最好的。

下面是 `Git-SVN` 通用的操作



```sh
#Download an SVN project and its entire code history and initialize it as a git code base
$ git svn clone -s [repository]

#View the current version Library
$ git svn info

#Retrieve changes from all branches of the remote warehouse
$ git svn fetch

#Retrieve the changes of the current branch of the remote warehouse and merge it with the local branch
$ git svn rebase 

#Upload the local warehouse of the current branch to the remote warehouse
$ git svn dcommit

#Pull new branch and submit to remote warehouse
$ git svn copy [remote_branch] [new_remote_branch] -m [message]

#Create local branch corresponding to remote branch
$ git checkout -b [local_branch] [remote_branch]
```





# 参考

[官方英文文档](https://git-scm.com/docs/git-svn)

[Compared with GIT and SVN, this article is easy to understand](https://developpaper.com/compared-with-git-and-svn-this-article-is-easy-to-understand/)

[The Best of Git working with Subversion](https://www.taringamberini.com/en/blog/howto/the-best-of-git-working-with-subversion/)

[The Dream of a Bi-directional Git-SVN mirror](https://blog.tfnico.com/2011/03/dream-of-bi-directional-git-svn-mirror.html)