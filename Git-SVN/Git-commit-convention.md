
<!-- TOC -->

- [1. Git Commit 规范约定](#1-git-commit-规范约定)
  - [1.1. 背景](#11-背景)
  - [1.2. Commit message 结构](#12-commit-message-结构)
  - [1.3. Commit 约定遵循准则](#13-commit-约定遵循准则)
- [2. 参考](#2-参考)

<!-- /TOC -->


# 1. Git Commit 规范约定

## 1.1. 背景

Git 每次提交代码都需要写 commit message，否则就不允许提交。一般来说，commit message 应该清晰明了，说明本次提交的目的，具体做了什么操作，但是在日常开发中，大家的 commit message 千奇百怪，中英文混合使用、fix bug 等各种笼统的 message 司空见怪，这就导致后续代码维护成本特别大，有时自己都不知道自己的 fix bug 修改的是什么问题。基于以上这些问题，我们希望通过某种方式来监控用户的 git commit message，让规范更好的服务于质量，提高大家的研发效率。


## 1.2. Commit message 结构

提交信息应该遵循下面的结构：
```
<type>[optional scope]: <description>

[optional body]

[optional footer(s)]
```

type: 是必选项，表示类型。用于说明 git commit 的类别，只允许使用下面的标识。
- feat：新功能（feature）。
- fix/to：修复 bug，可以是 QA 发现的 BUG，也可以是研发自己发现的 BUG。
  - fix：产生 diff 并自动修复此问题。适合于一次提交直接修复问题。
  - to：只产生 diff 不自动修复此问题。适合于多次提交，最终修复问题提交时使用 fix。
- docs：文档（documentation）。
- style：格式。不影响代码运行的变动。
- refactor：重构，既不是新增功能，也不是修改 bug 的代码变动。
- perf：优化相关，比如提升性能、体验。
- test：增加测试。
- chore：构建过程或辅助工具的变动。
- revert：回滚到上一个版本。
- merge：代码合并。
- sync：同步主线或分支的 Bug。 

description: 必选项，提交信息的简短描述，不超过 50 个字符。写描述应遵循下面的规则：
- 建议使用中文，感觉中国人用中文描述问题能更清楚一些。
- 结尾不加句号或其他标点符号。


optional: 是可选项，提交时可以设置也可以不设置，由组内人员或开发者自行约定。
- scope: 用于说明 commit 发生变动的范围。比如数据层、控制层、视图层等等，视项目不同而不同。
  
  > 提交信息中带有 scope 的可选项时，后面的花括号应采用英文。
- body:
- footer: 脚本。

根据以上 git commit message 规范，编写的格式如下：
```
fix(DAO): 用户查询缺少username属性 
feat(Controller): 用户查询接口开发
```

## 1.3. Commit 约定遵循准则

- 每个提交都必须使用类型字段前缀，它由一个名词构成，诸如 feat 或 fix ， 其后接可选的范围字段，可选的 !范围，以及必要的冒号（英文半角）和空格。

- 当一个提交为应用或类库实现了新功能时，必须使用 feat 类型。

- 当一个提交为应用修复了 bug 时，必须使用 fix 类型。

- 范围字段可以跟随在类型字段后面。范围必须是一个描述某部分代码的名词，并用圆括号包围，例如： 

  ```
  fix(parser): xxxx
  ```

- 描述字段必须直接跟在 <类型>(范围) 前缀的冒号和空格之后。 描述指的是对代码变更的简短总结，例如： 

  ```
  fix: array parsing issue when multiple spaces were contained in string
  ```

- 在简短描述之后，可以编写较长的提交正文，为代码变更提供额外的上下文信息。正文必须起始于描述字段结束的一个空行后。

- 提交的正文内容自由编写，并可以使用空行分隔不同段落。

- 在正文结束的一个空行之后，可以编写一行或多行脚注。每行脚注都必须包含 一个令牌（token），后面紧跟 `:<space>` 或 `<space>#` 作为分隔符，后面再紧跟令牌的值（受 git trailer convention 启发）。

- 脚注的令牌必须使用 `-` 作为连字符，比如 Acked-by (这样有助于区分脚注和多行正文)。有一种例外情况就是 `BREAKING CHANGE`，它可以被认为是一个令牌。

- 脚注的值可以包含空格和换行，值的解析过程必须直到下一个脚注的令牌/分隔符出现为止。

- 破坏性变更必须在提交信息中标记出来，要么在 `<类型>(范围)` 前缀中标记，要么作为脚注的一项。

- 包含在脚注中时，破坏性变更必须包含大写的文本 BREAKING CHANGE，后面紧跟着冒号、空格，然后是描述，例如：

  ```
  BREAKING CHANGE: environment variables now take precedence over config files 
  ```

- 包含在 `<类型>(范围)` 前缀时，破坏性变更必须通过把 ! 直接放在 : 前面标记出来。 如果使用了 !，那么脚注中可以不写 BREAKING CHANGE:， 同时提交信息的描述中应该用来描述破坏性变更。

- 在提交说明中，可以使用 feat 和 fix 之外的类型，比如：

  ```
  docs: updated ref docs
  ```

- 工具的实现必须不区分大小写地解析构成约定式提交的信息单元，只有 `BREAKING CHANGE` 必须是大写的。

- `BREAKING-CHANGE` 作为脚注的令牌时必须是 `BREAKING CHANGE` 的同义词。



# 2. 参考
- [开源 Angular 项目 commit 约定规范](https://github.com/angular/angular/blob/main/CONTRIBUTING.md)
- [Conventional Commits](https://www.conventionalcommits.org/zh-hans/v1.0.0/)
- [阿里巴巴：约束 git commit 提交规范](https://mp.weixin.qq.com/s/vzgST0ko-HZVkFFiSZ2xGg)
- [How to Write a Git Commit Message](https://chris.beams.io/posts/git-commit/)
- [Git 修改已提交的commit注释](https://www.jianshu.com/p/098d85a58bf1)
- [Commit message 和 Change log 编写指南](http://www.ruanyifeng.com/blog/2016/01/commit_message_change_log.html)
- [git rebase vs git merge详解](https://www.cnblogs.com/kidsitcn/p/5339382.html)