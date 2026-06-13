<!--
 * @Author: JohnJeep
 * @Date: 2022-04-09 10:35:45
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-06-13 17:00:54
 * @Description: document convention
 * Copyright (c) 2025 by John Jeep, All Rights Reserved. 
-->

# 文档写作约定

- 中文与英文字母之间必须间隔一个空格；
- 文档中的所有中文均采用中文逗号和中文句号，且都为半角字符；
- 中文与英文之间的冒号均采用英文半角字符；
- 汉字与汉字之间的的花括号均采用中文半角字符，英文与英文之间均采用英文半角字符，汉字与英文之间采用英文半角字符；
- Markdown 英文标题的开头英文单词的首字母大写，后面均小写；
- 每行文本不超过 118，超过换行；


## 博客自动同步约定

- 同步范围由 `.blog-sync.json` 白名单控制，仅同步指定目录中的 `.md` 文件。
- 同步脚本为 `add_frontmatter.py`，产物目录为 `Blogs/source/_posts/synced`。
- 分类规则：按源文件目录层级自动生成 `categories`。
- 标签规则：按路径和文件名自动生成 `tags`。
- 每次提交后由 GitHub Actions 自动构建并发布。


## References

排版规范：
- [中文文案排版指北 - GitHub](https://github.com/sparanoid/chinese-copywriting-guidelines)
- [写给大家看的中文排版指南 - 知乎](https://zhuanlan.zhihu.com/p/20506092)
- [中文文案排版细则 - Dawner](https://dawner.top/posts/chinese-copywriting-rules/)
- [中文技术文档写作风格指南](https://github.com/yikeke/zh-style-guide/)

提 issue/question 推荐阅读资料：
- [《提问的智慧》](https://github.com/ryanhanwu/How-To-Ask-Questions-The-Smart-Way)
- [《如何向开源社区提问题](https://github.com/seajs/seajs/issues/545)
- [《如何有效地报告 Bug》](http://www.chiark.greenend.org.uk/~sgtatham/bugs-cn.html)
- [《如何向开源项目提交无法解答的问题》](https://zhuanlan.zhihu.com/p/25795393)
