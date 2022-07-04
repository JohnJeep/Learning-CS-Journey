# 简介

SMTP（Simple Mail Transfer Protocol）即简单邮件传输协议，尽管邮件服务器可以用SMTP发送、接收邮件，但是邮件客户端只能用 SMTP 发送邮件，接收邮件一般用 IMAP 或者 POP3。邮件客户端使用 TCP 的 25 号端口与服务器通信。

SMTP在1982年首次被定义在  RFC 821 ，在2008它被更新为扩展的SMTP协议，补充在文件 RFC 5321 ，扩展的协议是目前使用最广泛的协议。

# 命令

SMTP命令由 4 个不区分大小写的字母组成，如果命令带参数，则用空格与参数隔开，每条命令用回车换行结尾 `CRLF`。

## EHLO（Extended hello）or  HELO（hello）

这个命令用于说明自己是SMTP客户端身份，参数包含客户端的域名(domain)。其中EHLO是SMTP补充协议（ RFC 5321 ）中用于替换HELO命令的新命令，协议规定服务器支持EHLO命令的时候，尽量使用EHLO命令，为了兼容以前的版本，要求服务器继续支持HELO命令。如果收到回复OK，说明发送者和接收者处于初始状态，所有的状态表和缓存区都被清零。

## MAIL

这个命令的参数是发送者邮箱 <reverse-path>，参数中有 **FROM**关键字，这个命令会清空之前的发送者邮箱（the reverse-path buffer）、接收者邮箱（forward-path buffer）和邮件数据（the mail data buffer）。

## RCPT （recipient）

用于指定一个邮件接收者，参数中有TO 关键字，指定多个接收者通过重复使用这个命令。

## DATA

这个命令没有参数，告诉服务器接着要发送邮件内容。
邮件内容包含邮件标题项（message header section ）和邮件正文（message body），
标题项（Header Fields ）是以项目名（field name）为行的起点，接着是冒号(":")，跟着是内容（field body）以回车换行结束（CRLF），下面是标题项的例子
From: Bob@example.com 
To: Alice@example.com
Cc: theboss@example.com 

### subject: subject

其中From、To、Cc、subject就是项目名，冒号后是内容。邮件的标题区与正文区需要用一个**空行**隔开。两者共同组成DATA命令的参数，正文区用只有一个点字符 **.** 的单行来结束。

## SEND

初始化邮件事务，邮件数据被转发到一个或多个终端。

## SOML（SEND OR MAIL)

初始化邮件事务，邮件数据被转发到一个或多个终端或邮箱。

## SAML（SEND AND MAIL）

初始化邮件事务，邮件数据被转发到一个或多个终端和邮箱。

## RSET（RESET）

这个命令用来终止邮件事务（mail transaction），任何已经存储的发送者、接收者、邮件数据（mail data）信息都被丢弃，缓存区被清零。

## VRFY（VERIFY）

验证邮箱是否存在，如果参数是用户名，则返回一个全名（如果存在）。

## EXPN（EXPAND）

验证邮箱列表

## HELP

返回帮助信息，带参数时候，返回指定的帮助信息。

## NOOP

这个命令指示服务器收到命令后不用回复 “OK”

## QUIT

关闭传输通道。

## TURN

交换邮件发送者和接收者的角色，这个命令用在建立连接成本高的时候，TCP连接不用这个命令。这个命令会产生安全问题，只有在服务器可以被授权作为客户端时候才能用。

# 原理

SMTP被设计基于以下交流模型：当用户需要发邮件时候，邮件发送者(sender-SMTP)建立一个与邮件接收者(receiver-SMTP)通信的通道，发送者发送SMTP命令给接收者，接收者收到后对命令做回复。

通信通道被建立后，发送者发送 MAIL 命令来指定发送者的邮件，如果接受者接收这个邮件，就回复 OK ，接着发送者发送 RCPT命令来指定接收者的邮箱，如果被接收同样回复OK，如果不接受则拒绝（不会终止整个通话）。接收者邮箱确定后，发送者用DATA命令指示要发送数据，并用一个 .  结束发送。如果数据被接收，会收到OK ，然后用QUIT结束会话。

SMTP 发送邮件步骤

- 用 **MAIL** 命令给出发送者的身份
  
  这个命令告诉接收者，开始一个新的邮件事务，重置所有的状态表和缓存区，包括接受者信息和邮件数据，<reverse-path>被用于报告错误，如果命令被接受，返回250 OK

- 用一个或者多个**RCPT**命令给出接收者信息
  
  这个命令提供一个接收者邮箱，如果被接受返回250 OK，如果不能被识别，返回550 Failure，这个第二步可以被重复多次。

- 用 **DATA**命令给出邮件数据
  
  如果被接受，返回354，并认为所有后续行都会邮件数据信息。当收到文本结束符时候，返回 250 OK。邮件数据的末尾必须被指明，为了激活命令和回复的对话。通过发送只包含一个英文句号的行，来提示邮件数据结束。

参考

- [RFC 2821: Simple Mail Transfer Protocol](https://www.rfc-editor.org/rfc/rfc2821)

- [C语言使用SMTP发送邮件_yanglx2022的博客-CSDN博客_c语言发送邮件](https://blog.csdn.net/yanglx2022/article/details/47759069)

- [SMTP协议介绍_Ouyang_Lianjun的博客-CSDN博客_smtp](https://blog.csdn.net/qq_35644234/article/details/68961603)
  
      
