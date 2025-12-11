<!--
 * @Author: JohnJeep
 * @Date: 2019-04-04 23:28:59
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-04-04 19:21:21
 * @Description: Git commit Conventional
 * Copyright (c) 2025 by John Jeep, All Rights Reserved. 
-->
# 1. Git Commit 规范约定

## 1.1. 背景
Git 每次提交代码都需要写 commit message，否则就不允许提交。一般来说，commit message 应该清晰明了，
说明本次提交的目的，具体做了什么操作，但是在日常开发中，大家的 commit message 千奇百怪，
中英文混合使用、fix bug 等各种笼统的 message 司空见怪，这就导致后续代码维护成本特别大，
有时自己都不知道自己的 fix bug 修改的是什么问题。
基于以上这些问题，我们希望通过某种方式来监控用户的 git commit message，让规范更好的服务于质量，提高大家的研发效率。


## 1.2. Commit message 结构
提交信息应该遵循下面的结构：
```sh
<type>[optional scope]: <description>

[optional body]

[optional footer(s)]
```

type: 是必选项，表示类型。用于说明 git commit 的类别，只允许使用下面的标识。

- feat：新功能（a new feature）。
- fix/to：修复 bug，可以是 QA 发现的 BUG，也可以是研发自己发现的 BUG。
  - fix：产生 diff 并自动修复此问题。适合于一次提交直接修复问题。
  - to：只产生 diff 不自动修复此问题。适合于多次提交，最终修复问题提交时使用 fix。
- style：代码格式类的变更，不影响代码运行的变动。比如用格式化代码或删除空行等。
- refactor：既不是新增功能，也不是修改 bug 的代码变动。其它类代码的变动，例如简化代码、重命名变量、删除冗余代码等。
- chore：构建流程、依赖管理或辅助工具的变动。
- perf：代码的改变是为了提高新能、体验等。
- test：新增测试用例或更新现有测试用例。
- ci: 持续集成或部署相关的变动，例如修改 Jenkins、GitLab CI 等 CI 配置文件或更新系统单元文件。
- docs：只有文档（documentation）的改变。包括修改用户文档、开发文档。
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
  ```sh
  BREAKING CHANGE: environment variables now take precedence over config files 
  ```
- 包含在 `<类型>(范围)` 前缀时，破坏性变更必须通过把 ! 直接放在 : 前面标记出来。 如果使用了 !，那么脚注中可以不写 BREAKING CHANGE:， 同时提交信息的描述中应该用来描述破坏性变更。
- 在提交说明中，可以使用 feat 和 fix 之外的类型，比如：
  ```
  docs: updated ref docs
  ```
- 工具的实现必须不区分大小写地解析构成约定式提交的信息单元，只有 `BREAKING CHANGE` 必须是大写的。
- `BREAKING-CHANGE` 作为脚注的令牌时必须是 `BREAKING CHANGE` 的同义词。


# 版本规范管理

| **阶段**               | **全称**             | **特点**                               | **适用范围**      |
| :--------------------- | :------------------- | :------------------------------------- | :---------------- |
| Pre-Alpha              | Pre-Alpha            | 最早期，功能搭建中，不对外发布         | 内部研发自测      |
| Alpha                  | Alpha Version        | 功能不完整、不稳定，验证核心功能       | 内部测试团队      |
| Beta                   | Beta Version         | 功能基本完整，仍有 Bug，分封测/公测    | 小部分或公开用户  |
| RC                     | Release Candidate    | 候选版本，接近 GA，功能冻结            | 部分用户试用      |
| EA                     | Early Access         | 提前体验版本，收集早期反馈，可能不稳定 | 受邀用户/早期客户 |
| Preview / Tech Preview | 技术预览版           | 提前展示新功能，可能缺陷多             | 开发者/技术用户   |
| GA                     | General Availability | 正式版，功能完整稳定，全面推广         | 所有用户          |
| Patch / Hotfix         | -                    | 小更新，修复 Bug / 安全漏洞            | 已发布版本用户    |
| LTS                    | Long Term Support    | 长期支持，数年维护                     | 企业/稳定用户     |
| EOL                    | End of Life          | 生命周期结束，不再维护                 | -                 |

语义化版本 2.0.0：https://semver.org/lang/zh-CN/


# Merge Request 规范

| **分类**     | **规范要求**                                                 | **示例 / 模板**                                              |
| :----------- | :----------------------------------------------------------- | :----------------------------------------------------------- |
| **MR 标题**  | 使用 `[类型]: 简短描述` 格式，遵循 Conventional Commits      | `feat: 增加多路 V4L2 摄像头支持`  `fix: 修复 IMU 时间戳不同步问题` |
| **类型**     | `feat`（新功能）、`fix`（Bug 修复）、`docs`（文档）、`style`（格式）、`refactor`（重构）、`test`（测试）、`chore`（构建/依赖/CI） | `feat: 支持 DDS io-uring 封装`                               |
| **MR 描述**  | 推荐使用固定模板，包含变更内容、原因、关联 issue、测试情况   |                                                              |
| **提交规范** | 每个 commit 信息清晰可读，MR 合并时 **Squash** 成 1 个主干提交 | `git commit -m "fix: 修复摄像头初始化失败的问题"`            |
| **MR 大小**  | **小步提交**，单个 MR 不要太大，避免包含不相关修改           | 一个 MR 只做一件事：修 Bug / 加功能                          |
| **评审流程** | 至少 1~2 人 review 通过；作者修改后需 `resolve discussions`  | Reviewer 检查点：功能正确性 / 可维护性 / 性能风险 / 测试覆盖 / 文档更新 |
| **合并策略** | 默认使用 **Squash Merge**，保持主干简洁；必要时可用 Merge Commit；禁止 Rebase Merge（除非明确要求） | `Squash and merge`                                           |
| **Draft MR** | 未完成时用 `Draft:` 标题                                     | `Draft: feat: 支持 GPU 零拷贝跨进程通信`                     |


# 2. References

1. [Github Angular 项目 commit 约定规范](https://github.com/angular/angular/blob/main/CONTRIBUTING.md)
2. [Conventional Commits](https://www.conventionalcommits.org/zh-hans/v1.0.0/)
3. [阿里巴巴：约束 git commit 提交规范](https://mp.weixin.qq.com/s/vzgST0ko-HZVkFFiSZ2xGg)
4. [How to Write a Git Commit Message](https://chris.beams.io/posts/git-commit/)
5. [Git 修改已提交的commit注释](https://www.jianshu.com/p/098d85a58bf1)
6. [Commit message 和 Change log 编写指南](http://www.ruanyifeng.com/blog/2016/01/commit_message_change_log.html)
7. [git rebase vs git merge详解](https://www.cnblogs.com/kidsitcn/p/5339382.html)

   