<!--
 * @Author: JohnJeep
 * @Date: 2022-01-27 17:21:57
 * @LastEditors: JohnJeep
 * @LastEditTime: 2023-08-04 14:40:59
 * @Description: Typora 使用手册
 * Copyright (c) 2023 by John Jeep, All Rights Reserved. 
-->

# 1. Typora使用手册


## 1.1. Markdown 语法
-  段落：`Ctrl + 0`
- 一级到六级标题：`Ctrl + 1 到 Ctrl + 6`
- 加粗： `Ctrl/Cmd + B`
  - **测试**
- 斜体：`Ctrl + I `
  - *测试*
- 下划线：`Ctrl + U `
   - <u>测试</u>
- 删除线：`Shift + Alt + 5`
  - ~~ 测试 ~~
- 插入链接：`Ctrl/Cmd + K`
  - 支持超链接 ：[百度](https://www.baidu.com)
  - 本地链接：[本地](./)
  - 文章内锚点，制作文章目录：[第一章](#)
- 行内代码：`Ctrl/Cmd + Shift +` \`  
  `hello word!`
- 代码块： `Ctrl/Cmd + Shift + K`
  ```cpp
  #include <stdio.h>
  int main() {
      return 0;
  }
  ```
- 插入图片： `Ctrl/Cmd + Shift + I`
    ![bilibili](https://i0.hdslb.com/bfs/vc/fb9521333b8ea79d90bdfc6da31cf83c52d93ec9.png)
- 插入表格：`Ctrl + T`
  - 表格支持拖拽移动、网页端表格复制转换
    | 2    | 5    |      |
    | ---- | ---- | ---- |
    | 1    | 4    |      |
    | 3    | 6    |      |
- 无序列表： `Ctrl/Cmd + Shift + ]`
- 有序列表：`Ctrl/Cmd + Shift + [`
- 任务列表。任务列表使您可以创建带有复选框的项目列表。在支持任务列表的Markdown应用程序中，复选框将显示在内容旁边。要创建任务列表，请在任务列表项之前添加破折号 `-` 和方括号 `[ ]`，并在 `[ ]` 前面加上空格。要选择一个复选框，请在方括号 `[x]` 之间添加 `x` 。
  - [x] 001
  - [ ] 002
  - [ ] 003
- 引用：`Ctrl + Shift + Q`
  >   少年易老学难成，一寸光阴不可轻。
  >
  >   未觉池塘春草梦，阶前梧叶已秋声 。
  >
  >   ​                         —朱熹《劝学诗》
- LeTex 公式块，行内公式：`Shift + Ctrl + M`
  $$
    e^{i\pi} + 1 = 0
  $$
- 内联公式：$ e^{i\pi} + 1 = 0 $
- 文献引用  
  Linux，全称GNU/Linux，是一种免费使用和自由传播的[类UNIX](https://baike.baidu.com/item/类UNIX/9032872)操作系统[^1]。<br>
  [^1]: Linux防火墙脚本化管理研究．万方．2019[引用日期2019-08-04]
- 上标：X^3^
- 下标：H~2~O
- 高亮：==测试==
- 注释：`<!--这是一段注释-->`
- 分隔线（三条短横线）：---
- Emoji 图标。[表情符号短代码](https://www.webfx.com/tools/emoji-cheat-sheet/)，以冒号开头和结尾，并包含表情符号的名称。
  - :arrow_backward:
  - :aries:
  - :closed_book:
  - :fork_and_knife:
  - :joy:
- 目录生成：[TOC]
- 流程图
  ```flow
  ```

  ```sequence
  ```


## 1.2. 通用快捷键
- 撤销： `Ctrl/Cmd + Z`
- 切换原文和语法：`Ctrl/Cmd + /`
- 返回Typora顶部 ：`Ctrl + Home`
- 返回Typora底部 ：`Ctrl + End`
- 选中某句话 ：`Ctrl + L`
- 选中某个单词 ：`Ctrl + D`
- 搜索：`Ctrl + F` 
- 选中相同格式的文字：`Ctrl + E`
- 搜索并替换： `Ctrl + H`

    
## 1.3. 表格
<table><tr><td bgcolor="violet"> 
  背景色 浅紫色
</td></tr></table>


<table border="10">
<tr>
<th>表头1</th>
<th>表头2</th>
</tr>
<tr>
<td>row 1, cell 1</td>
<td>row 1, cell 2</td>
</tr>
<tr>
<td>row 2, cell 1</td>
<td>row 2, cell 2</td>
</tr>
</table>

# References
- Markdown 教程：https://markdown.com.cn/intro.html
- 在线生成表格代码：https://www.tablesgenerator.com

