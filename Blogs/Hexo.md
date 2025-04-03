---
title: Hexo 使用技巧
tags: [hexo, blog]
categories: blog
data: 2025-03-29 21:35:31
---

<!--
 * @Author: JohnJeep
 * @Date: 2025-03-28 10:19:40
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-04-03 10:33:35
 * @Description: how to use hexo 
 * Copyright (c) 2025 by John Jeep, All Rights Reserved. 
-->


# Install

```bash
# global install hexo 
npm install -g hexo-cli

# start 
$ hexo init <folder>   # folder before not create
$ cd <folder>
$ npm install

# install git
npm install hexo-deployer-git --save

# install butterfly theme
npm install hexo-theme-butterfly --save
```



默认页面
- 首页
- 归档页：默认的归档页（Archives）是按时间组织的
- 分类页
- 标签页


# 使用技巧

```bash
hexo clean      # 清除缓存文件 (db.json) 和已生成的静态文件 (public/)
hexo g          # 生成静态文件，hexo generate 的简写，只生成，不清理
hexo g --watch  # 能够监视文件变动并立即重新生成静态文件
hexo s          # 本地启动，hexo server 的简写
hexo s --debug  # 开发模式（自动检测文件变化）
hexo g -d       # 生成后立即部署
hexo d          # 部署

# 一键清理、生成、运行
hexo clean && hexo g && hexo s

# 生成后一键部署
hexo clean && hexo g && hexo d
```

# Front-matter

Front-matter：确保每个Markdown文件都有正确的Front-matter（标题、日期、分类等）

示例：
```bash
---
title: 你的文章标题
date: 2023-10-01  # 建议填写，否则 Hexo 可能不会正确排序
tags: [标签1, 标签2]
categories: 分类
---
```

# References

- hexo official: https://hexo.io/zh-cn/docs/
- Github Pages入门指南: https://sinoui.github.io/sinoui-guide/docs/github-pages-introduction