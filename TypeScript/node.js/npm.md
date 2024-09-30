# introduce

Node 是做事工具，npm 是安装工具

# npm command

```sh
npm -v                                   # 显示版本，检查npm 是否正确安装
npm install express                      # 安装express模块
npm install express@0.0.1                # 安装指定版本
npm install -g express                   # 全局安装express模块
npm install --save-dev xxx               # 安装包依赖到开发环境的本地工程中 
npm install --save-prod xxx              # 安装包依赖到生产环境的本地工程中 
npm list                                 # 列出已安装模块
npm show express                         # 显示模块详情
npm update                               # 升级当前目录下的项目的所有模块
npm update express                       # 升级当前目录下的项目的指定模块
npm update -g express                    # 升级全局安装的express模块
npm uninstall express                    # 删除指定的模块
```

# package.json

每个 JavaScript 项目（无论是 Node.js 还是浏览器应用程序）都可以被当作 npm 软件包，并且通过 package.json 来描述项目和软件包信息。

当运行 npm init 初始化 JavaScript/Node.js 项目时，将生成 package.json 文件，文件内的内容(基本元数据)由开发人员提供：

- name：JavaScript 项目或库的名称。
- version：项目的版本。通常，在应用程序开发中，由于没有必要对开源库进行版本控制，因此经常忽略这一块。但是，仍可以用它来定义版本。
- description：项目的描述。
- license：项目的许可证。
- scripts: 项目本地允许的命令行工具。


# package-lock.json

该文件描述了 npm JavaScript 项目中使用的依赖项的确切版本。如果 package.json 是通用的描述性标签，则 package-lock.json 是成分表。


# references
- [什么是nodeJs它和NPM关系与应用](https://www.cnblogs.com/kenx/p/17381772.html)
