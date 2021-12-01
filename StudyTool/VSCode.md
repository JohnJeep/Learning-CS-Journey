<!--
 * @Author: JohnJeep
 * @Date: 2019-09-11 13:46:17
 * @LastEditTime: 2021-12-01 23:35:34
 * @LastEditors: Windows10
 * @Description: VSCode常用快捷键说明
 * -->

# 1. VSCode 快捷键捷键

1. `Ctrl + P` 快速打开文件
2. `Ctrl+Shift+P` 或 `F1` 显示命令面板
3. `Ctrl+Shift+N` 打开一个新的窗口
4. `Ctrl+Shift+W` 关闭一个新的窗口
5. `Ctrl+N`  新建一个文件
6. `Ctrl+W` 关闭文件页面
7. `Ctrl + Shift + T` 重新打开已关闭的页面
8. Ctrl +` 打开或关闭终端集成终端
9. `Ctrl + Alt +  ← / →` 将当前页面分栏到左边或者右边
10. ` Ctrl + Shift + Home/End ` 删除光标右侧或左侧的所有内容
11. ` Ctrl + Backspace ` 删除上一个单词
12. ` Shift+Alt + ↓ / ↑ ` 向上或向下复制当前行
13. ` Alt + ↓ / ↑ ` 向上或向下移动当前行
14. `Ctrl+F2 ` 批量替换当前文件中所有匹配的文本
15. ` Ctrl + Alt + ↓ / ↑ ` 向上或向下添加光标
16. ` Alt + 数字 ` 直接跳到想要跳转的页面
17. `Ctrl+K+0`  折叠所有代码
18. `Ctrl+K+J`  展开所有代码
19. `Ctrl+Shift+[`  折叠光标所处代码块内的代码
20. `Ctrl+Shift+]`  展开光标所处代码块内的代码
20. `Ctrl Shift o` 当前文件中搜索 symbols（匹配到相关的）
20. `Ctrl T` 当前 workspace 中搜索 symbols

> 搜索 `process explorer` 查看进程资源管理


# 2. Config

官方 [variable-reference](https://code.visualstudio.com/docs/editor/variables-reference)
一些常见的变量名

```sh
${workspaceFolder} - VS Code当前打开工作区文件夹的路径
${file} - 当前打开文件的绝对路径
${fileBasename} - 当前打开文件的名称
${fileBasenameNoExtension} - 当前打开文件的名称，但是不加后缀名
${fileDirname} - 文件所在的文件夹路径
```


常用配置文件说明
- `tasks.json` (compiler build settings)
- `launch.json` (debugger settings)
- `c_cpp_properties.json` (compiler path and IntelliSense settings)



**[c_cpp_properties.json 官方文档](https://code.visualstudio.com/docs/cpp/c-cpp-properties-schema-reference)** 