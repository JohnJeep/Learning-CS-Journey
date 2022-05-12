/usr/include/curl/curl.h 中。

1 CURLcode curl_global_init(long flags);

描述：
这个函数只能用一次。(其实在调用curl_global_cleanup 函数后仍然可再用)
如果这个函数在curl_easy_init函数调用时还没调用，它讲由libcurl库自动完成。

参数：flags

CURL_GLOBAL_ALL  //初始化所有的可能的调用。
CURL_GLOBAL_SSL  //初始化支持 安全套接字层。
CURL_GLOBAL_WIN32 //初始化win32套接字库。
CURL_GLOBAL_NOTHING   //没有额外的初始化。


2 void curl_global_cleanup(void);

描述：在结束libcurl使用的时候，用来对curl_global_init做的工作清理。类似于close的函数。

3 char *curl_version( );

描述: 打印当前libcurl库的版本。


4 CURL *curl_easy_init( );

描述:
curl_easy_init用来初始化一个CURL的指针(有些像返回FILE类型的指针一样). 相应的在调用结束时要用curl_easy_cleanup函数清理.
一般curl_easy_init意味着一个会话的开始. 它的返回值一般都用在easy系列的函数中.

5 void curl_easy_cleanup(CURL *handle);

描述:
这个调用用来结束一个会话.与curl_easy_init配合着用. 

参数:
CURL类型的指针.

6 CURLcode curl_easy_setopt(CURL *handle, CURLoption option, parameter);

描述: 这个函数最重要了.几乎所有的curl 程序都要频繁的使用它.
它告诉curl库.程序将有如何的行为. 比如要查看一个网页的html代码等.
(这个函数有些像ioctl函数)

参数:
1 CURL类型的指针
2 各种CURLoption类型的选项.(都在curl.h库里有定义,man 也可以查看到)
3 parameter 这个参数 既可以是个函数的指针,也可以是某个对象的指针,也可以是个long型的变量.它用什么这取决于第二个参数.

CURLoption 这个参数的取值很多.具体的可以查看man手册.

7 CURLcode curl_easy_perform(CURL *handle);

描述:这个函数在初始化CURL类型的指针 以及curl_easy_setopt完成后调用. 就像字面的意思所说perform就像是个舞台.让我们设置的
option 运作起来.











# Refernce

Everything curl: https://everything.curl.dev/

官方 curl docs: https://curl.se/docs/

[curl_easy_setopt的基本选项解析](https://blog.csdn.net/whui19890911/article/details/79247062)

[curl学习篇3：curl API简介](https://blog.csdn.net/weixin_42645653/article/details/120200661)

