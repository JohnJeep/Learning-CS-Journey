<!--
 * @Author: JohnJeep
 * @Date: 2025-03-28 10:19:40
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-04-04 17:29:41
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

# install hexo-reference to support markdown footnotes
npm install hexo-reference --save
```



默认页面
- 首页
- 归档页：默认的归档页（Archives）是按时间组织的
- 分类页
- 标签页


# Hexo Command

```bash
hexo clean      # 清除缓存文件 (db.json) 和已生成的静态文件 (public/)
hexo g          # 生成静态文件，hexo generate 的简写，只生成，不清理
hexo g --watch  # 能够监视文件变动并立即重新生成静态文件
hexo s          # 本地启动，hexo server 的简写
hexo s --debug  # 开发模式（自动检测文件变化）
hexo g -d       # 生成后立即部署
hexo d          # 部署

hexo server -p 5000 # 更改端

# 一键清理、生成、运行
hexo clean && hexo g && hexo s

# 生成后一键部署
hexo clean && hexo g && hexo d
```

# Directory

使用 `hexo init <folder>` 初始化后，会生成下面的目录

- scaffolds

  模版文件
  
- source

  存放发表后的博客。markdown 文件在 `_post`目录下。

- theme
  
  博客使用的主题配置目录。
  
- node_modules

  hexo 安装需要的 npm 模块。

- _config.yml
  
  yml 语法格式的配置文件。
  
  注：yaml 语法中， `:`后面需要加上空格，不然会报错。
  
- package.json
  
  模块管理文件。



# Front-matter

Front-matter：确保每个 Markdown 文件都有正确的 Front-matter（标题、日期、分类等）

示例：

```bash
# Page Front-matter
---
title: xxxx       # 页面标题，必填
date: 2023-10-01  # 页面创建时间，必填
type: 'xxx'       # tags，categories，link 三个页面需要配置，必填
update: xxx       # 页面更新时间，可选
---

# Post Front-matter
---
title: Hello World        # 文章标题，必填
date: 2025/04/02 22:11    # 创建时间，必填
updated: 2025/04/02 22:11 # 更新时间，必填
tags: [c,cpp]             # 文章标签，可选
categories: cpp           # 文章分类，可选
---

```



# References

- hexo official: https://hexo.io/zh-cn/docs/
- Github hexo starter repo: https://github.com/hexojs/hexo-starter
- Github Pages: https://sinoui.github.io/sinoui-guide/docs/github-pages-introduction
- butterfly theme official: https://butterfly.js.org/
- Gtihub Hexo Filter MathJax: https://github.com/next-theme/hexo-filter-mathjax

---
Tools
- shields: https://shields.io
- unsplash: https://unsplash.com
- fa address: https://fontawesome.com/search
- 阿里巴巴矢量图标库: https://www.iconfont.cn

---
Blog
- 用『Hexo』搭建个人博客: https://liarrdev.github.io/post/Build-a-Blog-with-Hexo
- Hexo-Butterfly主题搭建记录: https://www.drflower.top/posts/5920b38e/
- 关于我 Butterfly 主题的所有美化: https://blog.imzjw.cn/posts/b74f504f