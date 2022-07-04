## API

`/usr/include/curl/curl.h` 中。

### CURLcode curl_global_init(long flags);

描述：
这个函数只能用一次。(其实在调用curl_global_cleanup 函数后仍然可再用)
如果这个函数在curl_easy_init函数调用时还没调用，它讲由libcurl库自动完成。

参数：flags

CURL_GLOBAL_ALL  //初始化所有的可能的调用。
CURL_GLOBAL_SSL  //初始化支持 安全套接字层。
CURL_GLOBAL_WIN32 //初始化win32套接字库。
CURL_GLOBAL_NOTHING   //没有额外的初始化。

### void curl_global_cleanup(void);

描述：在结束libcurl使用的时候，用来对curl_global_init做的工作清理。类似于close的函数。

### char *curl_version( );

描述: 打印当前libcurl库的版本。

# The Easy interface

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



### CURL *curl_easy_init( );

描述:
curl_easy_init用来初始化一个CURL的指针(有些像返回FILE类型的指针一样). 相应的在调用结束时要用curl_easy_cleanup函数清理.
一般curl_easy_init意味着一个会话的开始. 它的返回值一般都用在easy系列的函数中.

### void curl_easy_cleanup(CURL *handle);

描述:
这个调用用来结束一个会话.与curl_easy_init配合着用. 

参数:
CURL类型的指针.

### CURLcode curl_easy_setopt(CURL *handle, CURLoption option, parameter);

描述: 这个函数最重要了.几乎所有的curl 程序都要频繁的使用它.
它告诉curl库，程序将有如何的行为。比如要查看一个网页的html代码等.
(这个函数有些像ioctl函数)

参数:
1 CURL类型的指针
2 各种CURLoption类型的选项.(都在curl.h库里有定义,man 也可以查看到)
3 parameter 这个参数 既可以是个函数的指针,也可以是某个对象的指针,也可以是个long型的变量.它用什么这取决于第二个参数.

CURLoption 这个参数的取值很多.具体的可以查看man手册.

### CURLcode curl_easy_perform(CURL *handle);

描述:这个函数在初始化CURL类型的指针 以及curl_easy_setopt完成后调用. 就像字面的意思所说perform就像是个舞台.让我们设置的
option 运作起来.

## 

## Multi interface

接口使用的步骤

1. Create easy handles for transfers

2. Set options for transfers

3. Create multi handle

4. Set multi handle options

5. Drive all transfers if not all are completed

6. Wait for something to happen, goto(5)

7. Cleanup after transfer



## multi-socket interface

# The primary structs





## 命令

```ini
-a/–append    上传文件时，附加到目标文件
-A/–user-agent    设置用户代理发送给服务器
-anyauth    可以使用“任何”身份验证方法
-b/–cookie <name=string/file>    cookie字符串或文件读取位置
     –basic    使用HTTP基本验证
-B/–use-ascii    使用ASCII /文本传输
-c/–cookie-jar    操作结束后把cookie写入到这个文件中
-C/–continue-at    断点续传
-d/–data    HTTP POST方式传送数据
     –data-ascii    以ascii的方式post数据
     –data-binary    以二进制的方式post数据
     –negotiate    使用HTTP身份验证
     –digest    使用数字身份验证
     –disable-eprt    禁止使用EPRT或LPRT
     –disable-epsv    禁止使用EPSV
-D/–dump-header    把header信息写入到该文件中
     –egd-file    为随机数据(SSL)设置EGD socket路径
     –tcp-nodelay    使用TCP_NODELAY选项
-e/–referer    来源网址
-E/–cert <cert:[passwd]>    客户端证书文件和密码 (SSL)
     –cert-type    证书文件类型 (DER/PEM/ENG) (SSL)
     –key    私钥文件名 (SSL)
     –key-type    私钥文件类型 (DER/PEM/ENG) (SSL)
     –pass    私钥密码 (SSL)
     –engine    加密引擎使用 (SSL). “–engine list” for list
     –cacert    CA证书 (SSL)
     –capath    CA目录 (made using c_rehash) to verify peer against (SSL)
     –ciphers    SSL密码
     –compressed    要求返回是压缩的形势 (using deflate or gzip)
     –connect-timeout    设置最大请求时间
     –create-dirs    建立本地目录的目录层次结构
     –crlf    上传是把LF转变成CRLF
-f/–fail    连接失败时不显示http错误
     –ftp-create-dirs    如果远程目录不存在，创建远程目录
     –ftp-method [multicwd/nocwd/singlecwd]    控制CWD的使用
     –ftp-pasv    使用 PASV/EPSV 代替端口
     –ftp-skip-pasv-ip    使用PASV的时候,忽略该IP地址
     –ftp-ssl    尝试用 SSL/TLS 来进行ftp数据传输
     –ftp-ssl-reqd    要求用 SSL/TLS 来进行ftp数据传输
-F/–form <name=content>    模拟http表单提交数据
     –form-string <name=string>    模拟http表单提交数据
-g/–globoff    禁用网址序列和范围使用{}和[]
-G/–get    以get的方式来发送数据
-H/–header    自定义头信息传递给服务器
     –ignore-content-length    忽略的HTTP头信息的长度
-i/–include    输出时包括protocol头信息
-I/–head    只显示请求头信息
-j/–junk-session-cookies    读取文件进忽略session cookie
     –interface    使用指定网络接口/地址
     –krb4    使用指定安全级别的krb4
-k/–insecure    允许不使用证书到SSL站点
-K/–config    指定的配置文件读取
-l/–list-only    列出ftp目录下的文件名称
     –limit-rate    设置传输速度
     –local-port    强制使用本地端口号
-m/–max-time    设置最大传输时间
     –max-redirs    设置最大读取的目录数
     –max-filesize    设置最大下载的文件总量
-M/–manual    显示全手动
-n/–netrc    从netrc文件中读取用户名和密码
     –netrc-optional    使用 .netrc 或者 URL来覆盖-n
     –ntlm    使用 HTTP NTLM 身份验证
-N/–no-buffer    禁用缓冲输出
-o/–output    把输出写到该文件中
-O/–remote-name    把输出写到该文件中，保留远程文件的文件名
-p/–proxytunnel    使用HTTP代理
     –proxy-anyauth    选择任一代理身份验证方法
     –proxy-basic    在代理上使用基本身份验证
     –proxy-digest    在代理上使用数字身份验证
     –proxy-ntlm    在代理上使用ntlm身份验证
-P/–ftp-port
使用端口地址，而不是使用PASV
-q    作为第一个参数，关闭 .curlrc
-Q/–quote    文件传输前，发送命令到服务器
-r/–range    检索来自HTTP/1.1或FTP服务器字节范围
–range-file    读取（SSL）的随机文件
-R/–remote-time    在本地生成文件时，保留远程文件时间
     –retry    传输出现问题时，重试的次数
     –retry-delay    传输出现问题时，设置重试间隔时间
     –retry-max-time    传输出现问题时，设置最大重试时间
-s/–silent    静默模式。不输出任何东西
-S/–show-error    显示错误
     –socks4 <host[:port]>    用socks4代理给定主机和端口
     –socks5 <host[:port]>    用socks5代理给定主机和端口
     –stderr     
-t/–telnet-option <OPT=val>    Telnet选项设置
     –trace    对指定文件进行debug
     –trace-ascii    Like –跟踪但没有hex输出
     –trace-time    跟踪/详细输出时，添加时间戳
-T/–upload-file    上传文件
     –url    Spet URL to work with
-u/–user <user[:password]>    设置服务器的用户和密码
-U/–proxy-user <user[:password]>    设置代理用户名和密码
-w/–write-out [format]    什么输出完成后
-x/–proxy <host[:port]>    在给定的端口上使用HTTP代理
-X/–request    指定什么命令
-y/–speed-time    放弃限速所要的时间，默认为30
-Y/–speed-limit    停止传输速度的限制，速度时间
```

# Refernce

Everything curl: https://everything.curl.dev/

官方 curl docs: https://curl.se/docs/

[libcurl programming tutorial]([libcurl - programming tutorial](https://curl.se/libcurl/c/libcurl-tutorial.html))

[curl_easy_setopt的基本选项解析[libcurl - programming tutorial](https://curl.se/libcurl/c/libcurl-tutorial.html)](https://blog.csdn.net/whui19890911/article/details/79247062)

[curl学习篇3：curl API简介](https://blog.csdn.net/weixin_42645653/article/details/120200661)


