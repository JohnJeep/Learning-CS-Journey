

# 1. Git协议

Git 支持四种协议传输

- 本地(Local)协议
- Git协议
- HTTP协议
- SSH(Secure Shell)协议

## 1.1. SSH(Secure Shell)协议

SSH 协议支持口令与密钥两种安全验证模式，但无论那种模式，最终都需要使用密钥来加密数据以确保安全，而 SSH 密钥通常使用的算法为 RSA 和 DSA。

命令　

- SSH1：只支持RSAS算法
- SSH2：支持RSA和DSA算法  
- `ssh -T git@github.com` 查看SSHkey
- `sssh-keygen -t rsa` 使用RSA算法创建密钥
- `id_rsa` 密钥 和 `id_rsa.pub` 公钥

为什么要用SSH？

- 是保证本机(当前电脑)与GitHub服务器连接的有效凭证
- 因为GitHub需要识别出你推送的提交确实是你推送的，而不是别人冒充的，而Git支持SSH协议，所以，GitHub只要知道了你的公钥，就可以确认只有你自己才能推送。
- GitHub允许你添加多个Key，只要把每台电脑的Key都添加到GitHub，就可以在每台电脑上往GitHub推送了。
- Git支持多种协议，包括`https`，但通过`ssh`支持的原生git协议速度最快。

# 2. Git Internals

Git 不是存储每个文件与初始版本的差异，而是把数据看作是对小型文件系统的一组快照 (snapshots)。每次你提交更新，或在 Git 中保存项目状态时，它主要对当时的全部文件制作一个快照并保存这个快照的索引。 为了高效，如果文件没有修改，Git 不再重新存储该文件，而是只保留一个链接指向之前存储的文件。  

Git保证了数据的完整性。所有数据在存储前都计算校验和(SHA-1 散列)，然后以校验和来引用。Git 数据库中保存的信息都是以文件内容的哈希值来索引，而不是文件名。

## 2.1. Git 文件状态

- 已提交（committed）：表示数据已经安全的保存在本地仓库中。
- 已修改（modified）：表示修改了文件，但还没保存到本地仓库中。 
- 已暂存（staged）：表示对一个已修改文件的当前版本做了标记，存储到暂存区中。

## 2.2. .Git 目录组成

随着 Git 版本的不同，该目录下可能还会包含其他内容。 不过对于一个全新的 git init 版本库，这将是你看到的默认结构。 

- `description` 文件仅供 GitWeb 程序使用，我们无需关心。
- `config` 文件包含项目特有的配置选项。 
- `info` 目录包含一个全局性排除（global exclude）文件 ， 用以放置那些不希望被记录在` .gitignore` 文件中的忽略模式（ignored patterns）。 
- `hooks` 目录包含客户端或服务端的钩子脚本（hook scripts）。
- `objects` 目录存储所有的数据内容。
- `refs` 目录存储指向数据（分支、远程仓库和标签等）提交对象的指针。
- `index` 文件保存到暂存区中的信息。
- `HEAD` 文件指向目前被检出的分支。

## 2.3. Git objects

Git 是一个内容寻址文件系统，听起来很酷。但这是什么意思呢？ 这意味着，Git 的核心部分是一个简单的键值对数据库（key-value data store）。 你可以向 Git 仓库中插入任意类型的内容，它会返回一个唯一的键，通过该键可以在任意时刻再次取回该内容。

可以通过底层命令 `git hash-object` 来演示上述效果: 可将任意数据保存于 `.git/objects` 目录（即 对象数据库），并返回指向该数据对象的唯一的键。即计算对象 ID 并可选择性的从文件创建一个 blob（Compute object ID and optionally creates a blob from a file）。 

- `-w`：选项表示该命令不要只返回键，还要将该对象写入数据库中。
- `--stdin`：从标准输入读 object，而不是从文件中读。若不指定此选项，则须在命令尾部给出待存储文件的路径。

```bash
$ echo "hello" | git hash-object   --stdin -w
ce013625030ba8dba906f756967f9e9ca394464a

$ ls -a .git/objects/
./  ../  ce/  info/  pack/

$ git cat-file -p ce013625030ba8dba906f756967f9e9ca394464a
hello

$ git cat-file -t ce013625030ba8dba906f756967f9e9ca394464a
blob
```

此命令输出一个长度为 40 个字符的校验和。 这是一个 SHA-1 哈希值——一个将待存储的数据外加一个头部信息（header）一起做 SHA-1 校验运算而得的校验和。

```bash
$ find .git/objects/ -type f
.git/objects/ce/013625030ba8dba906f756967f9e9ca394464a
.git/objects/info/packs
```

再次查看 `.git/objects` 目录，那么可以在其中找到一个与新内容对应的文件。 这就是开始时 Git存储内容的方式——**一个文件对应一条内容**， 以该内容加上特定头部信息一起的 SHA-1 校验和为文件命名。 **校验和的前 2 个字符用于命名子目录，余下的 38 个字符则用作文件名。**

查看文件为 sha-1 值中内容: 

```bash
$ git cat-file -p 007105d165b1e388febc15f648156fd0ec3bb53f
tree 49fea036e5503edada62e2cd82bf2d25c3d6d2be
parent 2dda59cea1dc91a74101ef1fd0f30d7fc38919d9
author xxx <xxx@gmail.com> 1612772353 +0800
committer xxx <xxx@gmail.com> 1612772353 +0800
Update C++ STL
```

`git cat-file -t` 命令，查看 Git 内部存储对象的类型，只要给定该对象的 SHA-1 值：

```bash
$ git cat-file -t 007105d165b1e388febc15f648156fd0ec3bb53f
commit
```

Git 仓库中有五个对象（object）：三个 blob 对象（保存着文件快照）、一个 树对象（记录着目录结构和 blob 对象 index）以及一个提交对象（包含着指向上次提交对象 （父对象）的指针和所有提交信息）。

- 数据对象（blob object）：也叫文件对象，保存着文件的快照。

- 树对象（tree object），它能解决文件名保存的问题，也允许我们将多个文件组织到一起。 Git 以一种类似于 UNIX 文件系统的方式存储内容，但作了些许简化。 所有内容均以树对象和数据对
  象的形式存储，其中树对象对应了 UNIX 中的目录项，数据对象则大致上对应了 inodes 或文件内容。 一个树对象包含了一条或多条树对象记录（tree entry），每条记录含有一个指向数据对象或者子树对象的 SHA-1 指针，
  以及相应的模式、类型、文件名信息。
  - `git update-index` ：为文件创建一个暂存区。
  - `git write-tree` ：将暂存区内容写入一个树对象。
  - `git read-tree` ：把树对象读入暂存区。
    - `--prefix` 选项，将一个已有的树对象作为子树读入暂存区`。

- 提交对象（commit object）：包含着指向树对象（tree object）的指针和所有提交信息。每一个提交在 Git 中都通过 git 提交对象（git commit object）存储，该对象具有一个全局唯一的名称，叫做 `revision hash`。它的名字是由 SHA-1 算法生成，形如"998622294a6c520db718867354bf98348ae3c7e2"，我们通常会取其缩写方便使用，如"9986222"。
  - 对象构成：commit 对象中包含了 author 和 commit message 等内容。
  - 对象存储：`commit object` 保存一次变更提交内的所有变更内容，而不是增量（delta）变化的数据（很多人都理解错了这一点），所以 Git 对于每次改动存储的都是全部状态的数据。
  - 大对象存储：因对于大文件的修改和存储，同样也是存储全部状态的数据，所以可能会影响 Git 使用时的性能(glfs 可以改进这一点）。

- 提交树：多个 commit 对象会组成一个提交树，它让我们可以轻松的追溯 commit 的历史，可以用来对比提交树上提交信息（commit）之间变化的差异。

>  用 `git add` 和 `git commit` 命令时，Git 所做的工作实质就是将被改写的文件保存为数据对象，更新暂存区，记录树对象，最后创建一个指明了顶层树对象和父提交的提交对象。 


### 文件模式

Git 有三种文件模式。
- 100644，表明这是一个普通文件。
- 100755，表示一个可执行文件。
- 120000，表示一个符号链接。


## 2.4. Git HEAD

Git 中用 `reference` 或简写`refs` 这个指针来替代原始提交文件的 `SHA-1` 值。

HEAD reference：是一个符号引用（symbolic reference），并不完全是指向目前所在的分支, 而是指向正在操作的 `commit` 提交。

> 符号引用：表示它是一个指向其他引用的指针。 

- `cat .git/HEAD` 或者`git symbolic-ref HEAD`  查看HEAD文件中的内容
- `git show HEAD` 查看HEAD信息

当前活跃的分支在哪儿，`HEAD` 就指向哪儿，是git内部用来追踪当前位置的方式。 `HEAD` 并非只能指向分支的最顶端（时间节点距今最近的那个），它也可以指向任何一个节点。

当 HEAD 指针直接指向提交时，就会导致 `detached HEAD` 状态，即游离状态。在此状态下创建了新的提交，新提交的信息不属于任何分支。相对应的，现存的所有分支也不会受 `detached HEAD` 状态提交的影响。两种情况会导致`detached HEAD`，即游离状态。

- `git checkout --detach ` HEAD 直接脱离分支头指针，指向分支头指针指向的 commit
- `git checkout <commit id> ` 直接切换到commit id号

其它几种HEAD文件

- `ORIG_HEAD` 当使用一些在 Git 看来比较危险的操作去移动 HEAD 指针的时候，ORIG_HEAD 就会被创建出来，记录危险操作之前的 HEAD，方便恢复HEAD。可以看作一个修改 HEAD 之前的简单备份。
- `FETCH_HEAD` 记录从远程仓库拉取的记录。
- `MERGE_HEAD` 当运行 `git merge` 时，`MERGE_HEAD` 记录你正在合并到你的分支中的提交。`MERGE_HEAD`在合并的时候会出现，合并结束时就删除了这个文件。
- `CHERRY_PICK_HEAD` 记录您在运行 `git cherry-pick` 时要合并的提交。这个文件只在 `cherry-pick` 期间存在。



# 3. 学习参考

- [Git for Computer Scientists](https://eagain.net/articles/git-for-computer-scientists/): 简短的解释Git的数据模型，有很多的图来阐述。
- [How to explain git in simple words?](https://xosh.org/explain-git-in-simple-words/): 解释了Git底层实现的一些过程。
- [Git from the Bottom Up](https://jwiegley.github.io/git-from-the-bottom-up/): 不仅解释了Git的数据模型，还解释了其实现的细节。
- [Git PAT 使用](https://blog.csdn.net/yjw123456/article/details/119696726): Github 支持 personal access token 的用法。
- [A successful Git branching model](https://nvie.com/posts/a-successful-git-branching-model/)： Vincent Driessen 介绍 git 工作流。