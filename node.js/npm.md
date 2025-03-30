<!--
 * @Author: JohnJeep
 * @Date: 2020-09-05 23:42:59
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-03-31 00:58:58
 * @Description: npm learning
 * Copyright (c) 2025 by John Jeep, All Rights Reserved. 
-->

# 1. introduce

npm(Node Package Manager) 是 Node.js 的标准包管理器。


# 2. npm command

```bash
npm install                              # 安装 package.json 中所有的 dependency 
npm -v                                   # 显示版本，检查npm 是否正确安装
npm install express                      # 安装express模块
npm install express@0.0.1                # 安装指定版本
npm install -g express                   # 全局安装express模块
npm uninstall <package-name>             # 全局卸载指定的 pckage
npm install <package-name> -D            # 安装包依赖到开发环境中，生产环境不会用到 
npm install --save-prod xxx              # 安装包依赖到生产环境的本地工程中 
npm list                                 # 列出已安装模块
npm show <package-name>                  # 显示 package 信息
npm update                               # 升级当前目录下的项目的所有 package
npm update <package-name>                # 升级当前目录下的项目的指定 package
npm update -g <package-name>             # 升级全局安装的 package
npm install <package-name>@<version>     # 指定软件包的特定版本，方便进行版本控制
```

# package

package 是基于内置模块封装出来的，提供了更高级、更方便的 API，极大的提高了开发效率。

```bash
# 快速创建 package.json 文件
npm init -y 

```

`package.json` 中的 `dependencies` 节点，专门记录使用 `npm install` 命令安装了哪些 package。




# 3. package.json

每个 JavaScript 项目（无论是 Node.js 还是浏览器应用程序）都可以被当作 npm 软件包，并且通过 package.json 来描述项目和软件包信息。简单来说 `package.json` 是管理需要安装的所有依赖文件。


当运行 npm init 初始化 JavaScript/Node.js 项目时，将生成 package.json 文件，文件内的内容由开发人员提供：

- name：JavaScript 项目或库的名称。
- version：项目的版本。通常，在应用程序开发中，由于没有必要对开源库进行版本控制，因此经常忽略这一块。但是，仍可以用它来定义版本。
- description：项目的描述。
- license：项目的许可证。
- scripts: 项目本地允许的命令行工具。


# 4. package-lock.json

该文件描述了 npm JavaScript 项目中使用的依赖项的确切版本。如果 package.json 是通用的描述性标签，则 package-lock.json 是成分表。



# 5. references

- npm official: https://www.npmjs.com
- nodejs of npm: https://nodejs.cn/en/learn/getting-started/an-introduction-to-the-npm-package-manager
