<!--
 * @Author: JohnJeep
 * @Date: 2022-05-10 09:36:06
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-06-27 21:55:50
 * @Description: curl usage
 * Copyright (c) 2024 by John Jeep, All Rights Reserved. 
-->

# 1. Curl

linux curl 是一个利用 URL 规则在命令行下工作的文件传输工具。它支持文件的上传和下载，所以是综合传输工具，但按传统，习惯
称 url 为下载工具。


# 2. CLI

常用几个参数个参数的含义：

- `-f` 请求失败时不输出错误页面内容
- `-s` 静默模式，不显示进度条
- `-S` 静默模式下如果出错仍然显示错误
- `-L` 自动跟随重定向

利用这些参数实现一行命令完成复杂安装步骤，比如：

```bash
curl -fsSL https://claude.ai/install.sh | bash
```



# 3. API

`/usr/include/curl/curl.h` 中。

- `CURLcode curl_global_init(long flags);`

  描述：
  这个函数只能用一次。(其实在调用 curl_global_cleanup 函数后仍然可再用)
  如果这个函数在 curl_easy_init 函数调用时还没调用，它讲由 libcurl 库自动完成。

​	参数：flags

​	CURL_GLOBAL_ALL  //初始化所有的可能的调用。
CURL_GLOBAL_SSL  //初始化支持 安全套接字层

​	CURL_GLOBAL_WIN32 //初始化 win32 套接字库。
CURL_GLOBAL_NOTHING   //没有额外的初始化。

- `void curl_global_cleanup(void);`

​	描述：在结束 libcurl 使用的时候，用来对 curl_global_init 做的工作清理。类似于 close 的函数。

- `char *curl_version( );`

描述: 打印当前 libcurl 库的版本。

# 4. The Easy interface

同步接口。

```c
curl_easy_init()
curl_easy_cleanup()
curl_easy_setopt()
curl_easy_perform()
curl_easy_getinfo()
```

Easy 接口使用的步骤：

- Create easy handle for transfer
- Set options for transfer
- Perform transfer
- Cleanup after transfer



- `CURL *curl_easy_init( );`

  描述:
  curl_easy_init 用来初始化一个 CURL 的指针(有些像返回 FILE 类型的指针一样). 相应的在调用结束时要用 curl_easy_cleanup
  函数清理.
  一般 curl_easy_init 意味着一个会话的开始. 它的返回值一般都用在 easy 系列的函数中.

- `void curl_easy_cleanup(CURL *handle);`

  描述:
  这个调用用来结束一个会话.与 curl_easy_init 配合着用。

  参数:
  CURL 类型的指针.

- `CURLcode curl_easy_setopt(CURL *handle, CURLoption option, parameter)`

  描述: 这个函数最重要了.几乎所有的 curl 程序都要频繁的使用它.
  它告诉 curl 库，程序将有如何的行为。比如要查看一个网页的 html 代码等.
  (这个函数有些像 ioctl 函数)

  参数:
  1 CURL 类型的指针
  2 各种 CURLoption 类型的选项.(都在 curl.h 库里有定义,man 也可以查看到)
  3 parameter 这个参数 既可以是个函数的指针,也可以是某个对象的指针,也可以是个 long 型的变量.它用什么这取决于第二个参数。

  CURLoption 这个参数的取值很多.具体的可以查看 man 手册。

- `CURLcode curl_easy_perform(CURL *handle);`

  描述：这个函数在初始化 CURL 类型的指针 以及 curl_easy_setopt 完成后调用. 就像字面的意思所说 perform
  就像是个舞台.让我们设置的
  option 运作起来。



# 5. Multi interface

异步接口。

接口使用的步骤

1. Create easy handles for transfers

2. Set options for transfers

3. Create multi handle

4. Set multi handle options

5. Drive all transfers if not all are completed

6. Wait for something to happen, goto(5)

7. Cleanup after transfer



# 6. multi-socket interface



# 7. References
- offical curl docs: https://curl.se/docs/
- Everything curl: https://everything.curl.dev/
- [libcurl programming tutorial]([libcurl - programming tutorial](https://curl.se/libcurl/c/libcurl-tutorial.html))
- [libcurl - programming tutorial](https://curl.se/libcurl/c/libcurl-tutorial.html)
- [curl_easy_setopt 的基本选项解析 ](https://blog.csdn.net/whui19890911/article/details/79247062)
- [curl 学习篇 3：curl API 简介](https://blog.csdn.net/weixin_42645653/article/details/120200661)

